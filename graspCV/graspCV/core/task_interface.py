import os
import torch
import numpy as np
from numpy import ndarray
from typing import Tuple, List, Union

import onnxruntime
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

from configs import MODEL_SAVE_DIR, MODEL_PREFIX
from core.logger import logger
from core.utils import dtype_to_size, dims_to_size


class TaskInterface(object):
    def __init__(self, model_path: str,
                 input_shape: Union[Tuple, List],   # [height, width]
                 num_classes: int = None,
                 backend: str = "pytorch",
                 target_map=None,
                 debug=False):
        assert backend in ["pytorch", "onnxruntime", "tensorrt", "ncnn"], \
            f"get an unsupported backend '{backend}', supported list: {' '.join(backend)}"

        self.input_shape = input_shape      # [h, w]
        self.num_classes = num_classes
        self.model_path = self.model_path = os.path.join(MODEL_SAVE_DIR, MODEL_PREFIX, model_path)

        self.backend = backend
        self.target_map = target_map
        self.debug = debug
        if model_path:
            assert os.path.exists(self.model_path) and os.path.isfile(self.model_path)
            self._load_model()
        else:
            logger.warning("model not loading in base class 'TaskInterface'")

    def __call__(self, image: ndarray, **kwargs):
        return self.task_predict(image, **kwargs)

    # can override
    def task_predict(self, image: ndarray, **kwargs):
        return self.predict(image, **kwargs)

    # cann't override
    def predict(self, image: ndarray, **kwargs) -> Union[List, bool]:
        img = self._preprocess(image.copy())

        if self.backend == "pytorch":
            outputs = self._infer_pytorch(img, **kwargs)
            return self._post_process(outputs, image=image, **kwargs)
        elif self.backend == "onnxruntime":
            outputs = self.model.run(None, {self.model.get_inputs()[0].name: img})
            return self._post_process(outputs, image=image, **kwargs)
        elif self.backend == "tensorrt":
            outputs = []
            # copy input data to device
            cuda.memcpy_htod_async(self.inputs[0]["input"], img.astype(trt.nptype(self.inputs[0]["dtype"])), self.stream)
            # do inference
            self.context.execute_async_v3(self.stream.handle)
            # copy output data to host
            for output in self.outputs:
                output_h = np.empty(output["out_shape"], dtype=trt.nptype(output["dtype"]))
                cuda.memcpy_dtoh_async(output_h, output["output"], self.stream)
                outputs.append(output_h)
            self.stream.synchronize()
            return self._post_process(outputs, image=image, **kwargs)
        elif self.backend == "ncnn":
            # TODO: do ncnn inference
            # outputs = (model run)
            # return self._post_process(outputs, **kwargs)
            raise NotImplementedError
        else:
            raise RuntimeError(f"get an unsupported backend '{self.backend}'")

    # can override
    def _infer_pytorch(self, image: ndarray, **kwargs):
        if isinstance(image, ndarray):
            image = torch.from_numpy(image)
        outputs = self.model(image.to(self.model.device))
        return outputs

    # must override
    def _preprocess(self, image: ndarray) -> ndarray:
        raise NotImplementedError

    # must override
    def _post_process(self, outputs, **kwargs) -> Union[List, bool]:
        """
        params:
            outputs: any
        return:
            list[dict]:
                format is:
                    [{"target_id": cls, "grasp_scores": [score], "grasp_rt": [rt]}, ...]

                    one dict for one object, 'target_id' is int and align with TARGET_MAP's id, 'grasp_scores' is list[float],
                    'grasp_rt' is list[ndarray(4x4)]
        """
        raise NotImplementedError

    # can't override
    def _load_model(self):
        if self.backend == "pytorch":
            self.model = self._load_model_pytorch()
        elif self.backend == "onnxruntime":
            self.model = onnxruntime.InferenceSession(self.model_path)
        elif self.backend == "tensorrt":
            self._load_model_tensorrt()
        elif self.backend == "ncnn":
            self.model = self._load_model_ncnn()
        else:
            raise RuntimeError(f"get an unsupported backend '{self.backend}'")

    # must override when using pytorch backend
    def _load_model_pytorch(self):
        raise NotImplementedError

    def _load_model_tensorrt(self):
        self.inputs = []
        self.outputs = []

        with trt.Logger(trt.Logger.WARNING) as trt_logger, trt.Runtime(trt_logger) as runtime:
            with open(self.model_path, 'rb') as f:
                self.engine = runtime.deserialize_cuda_engine(f.read())
            self.context = self.engine.create_execution_context()

            self.stream = cuda.Stream()

            for io_tensor_id in range(self.engine.num_io_tensors):
                name = self.engine.get_tensor_name(io_tensor_id)
                if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                    in_shape = self.engine.get_tensor_shape(name)
                    dtype = self.engine.get_tensor_dtype(name)
                    nBytes = dims_to_size(in_shape) * dtype_to_size(dtype)
                    # nBytes = self.engine.get_tensor_bytes_per_component(name)
                    input_d = cuda.mem_alloc(nBytes)
                    self.inputs.append({"input": input_d,
                                        "nBytes": nBytes,
                                        "dtype": dtype})
                elif self.engine.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT:
                    out_shape = self.engine.get_tensor_shape(name)
                    dtype = self.engine.get_tensor_dtype(name)
                    nBytes = dims_to_size(out_shape) * dtype_to_size(dtype)
                    output_d = cuda.mem_alloc(nBytes)
                    self.outputs.append({"output": output_d,
                                         "nBytes": nBytes,
                                         "out_shape": out_shape,
                                         "dtype": dtype})

    def _load_model_ncnn(self):
        # TODO: load ncnn model
        raise NotImplementedError

    def release(self):
        if self.backend == "pytorch":
            del self.model
        elif self.backend == "tensorrt":
            self.engine.destroy()
            self.context.destroy()
            self.stream.destroy()
        elif self.backend == "onnxruntime":
            del self.model
        elif self.backend == "ncnn":
            pass

