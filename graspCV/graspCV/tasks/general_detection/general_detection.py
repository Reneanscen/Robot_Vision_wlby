import os
from abc import ABC

import cv2
import math
import numpy as np
from core import TaskInterface
from core.logger import logger
from typing import Union, Tuple, List, Dict
from scipy.spatial.transform import Rotation
from tasks.general_detection.detection import YoloDetection
from tasks.general_detection.depth_complete import DepthComplete
from tasks.general_detection.utils import get_coarse_or_fine_location, vis_object
from configs import FINE_LOC, TRIPLE_LOC, ONCE_LOC, REAL_SENSE_D435i_CAM_INTRINSIC, ROBOT_HAND_BASE_FOR_CUP, OBJ_BIAS,\
    ROBOT_HAND_BASE_FOR_STORAGE, K4A_CAM_INTRINSIC, TARGET_MAP, MODEL_SAVE_DIR, MODEL_PREFIX, FINE_LOC_GET_MULTI_POSE


class GeneralDetection(TaskInterface, ABC):
    def __init__(self,
                 detect_model_path: str,
                 depth_refine_model_path: str,
                 input_shape: Union[Tuple, List] = (640, 640),  # [height, width]
                 num_classes: int = None,
                 backend: str = "tensorrt",
                 target_map: Dict = None,
                 rotate_degree: int = 30,
                 mask_filter_thresh: int = 100,
                 debug: bool = False,
                 score_thresh: float = 0.5,
                 iou_thresh: float = 0.45):
        super(GeneralDetection, self).__init__("", input_shape, num_classes, backend, target_map, debug)
        self.yolo = None
        self.dc = None
        self.detect_model_path = os.path.join(MODEL_SAVE_DIR, MODEL_PREFIX, detect_model_path)
        self.depth_refine_model_path = os.path.join(MODEL_SAVE_DIR, MODEL_PREFIX, depth_refine_model_path)
        self._load_model()

        self.score_thresh = score_thresh
        self.iou_thresh = iou_thresh
        self.rotate_degree = rotate_degree
        self.mask_filter_thresh = mask_filter_thresh
        self.debug = debug
        self.robot_hand_base_for_cup = np.asarray(ROBOT_HAND_BASE_FOR_CUP, dtype=np.float32)
        self.robot_hand_base_for_storage = np.asarray(ROBOT_HAND_BASE_FOR_STORAGE, dtype=np.float32)        

    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        return image

    def _infer_pytorch(self, image: np.ndarray, **kwargs):
        depth = kwargs.get("depth", None)
        det_result = self.yolo(image)
        depth_refine = self.dc(image, depth)
        outputs = [det_result, depth_refine]
        return outputs

    def _post_process(self, outputs, **kwargs) -> Union[List, bool]:
        depth = kwargs.get("depth", None)
        image = kwargs.get("image", None)
        task_id = kwargs.get("task_id", None)
        target_id = kwargs.get("target_id", None)
        robot_extrinsic = kwargs.get("robot_extrinsic", None)
        det_results = outputs[0]
        depth_refine = outputs[1]
        class_map = TARGET_MAP[task_id]
        cam_intrinsic = K4A_CAM_INTRINSIC if (task_id == 0 and target_id == 0) else REAL_SENSE_D435i_CAM_INTRINSIC
        cam_intrinsic = np.asarray(cam_intrinsic, dtype=np.float32)
        image = vis_object(image, det_results)
        if self.debug:
            cv2.imshow("vis debug", image)
            cv2.waitKey(1)

        pose_list = list()
        for cls_name in det_results.keys():
            logger.info(f"get {cls_name}")
            det_result = det_results[cls_name]
            for det in det_result:
                mask = det["mask"]
                confidence = det["conf"]
                if confidence > self.score_thresh:

                    if cls_name in FINE_LOC:
                        logger.info("fine location")
                        pose, stat, cv_image = get_coarse_or_fine_location(image, mask, depth, cam_intrinsic,
                                                                           robot_extrinsic, mask_area_thresh=100,
                                                                           fine=True, coarse_loc_time=0,
                                                                           rotate_degree=0,
                                                                           robot_hand_base=self.robot_hand_base_for_cup,
                                                                           in_meter=True)
                        if cls_name in OBJ_BIAS:
                            pose = self.add_bias_to_xyz(pose, OBJ_BIAS[cls_name])
                            pose[1, 3] -= 0.03

                        if cls_name in FINE_LOC_GET_MULTI_POSE:
                            pose = self.get_rotation_for_obj(self.robot_hand_base_for_cup, pose[0:3, 3])
                        loc_type = 1

                    elif cls_name in TRIPLE_LOC:
                        logger.info("triple coarse location")
                        pose, stat, image = get_coarse_or_fine_location(image, mask, depth_refine, cam_intrinsic,
                                                                        robot_extrinsic, mask_area_thresh=100,
                                                                        fine=False, coarse_loc_time=3,
                                                                        rotate_degree=30,
                                                                        robot_hand_base=self.robot_hand_base_for_storage,
                                                                        in_meter=True)
                        loc_type = 0
                    elif cls_name in ONCE_LOC:
                        logger.info("one coarse location")
                        pose, stat, image = get_coarse_or_fine_location(image, mask, depth, cam_intrinsic,
                                                                        robot_extrinsic, mask_area_thresh=100,
                                                                        fine=False, coarse_loc_time=1,
                                                                        rotate_degree=90,
                                                                        robot_hand_base=self.robot_hand_base_for_cup,
                                                                        in_meter=True)
                        loc_type = 0
                    else:
                        logger.warning("Can't process this class type")
                        continue

                    res = {
                        "grasp_scores": [float(confidence)],
                        "grasp_rt": pose if isinstance(pose, list) else [pose],
                        "target_id": class_map[cls_name],
                        "loc_type": loc_type,
                    }
                    logger.info(f"{cls_name} info : {res}")
                    pose_list.append(res)

        return pose_list

    def _load_model_pytorch(self):
        logger.info("load yolo and depth complete models")
        self.yolo = YoloDetection(self.detect_model_path)

        configs = {"type": "tode",
                   "use_cuda": True,
                   "image_size": [320, 240],  # h, w
                   "target_size": [1280, 720],
                   "depth_min": 0.3,
                   "depth_max": 1.5,
                   "depth_norm": 1000,
                   "params": {"lambda_val": 1, "res": True},
                   "weight_path": self.depth_refine_model_path}
        self.dc = DepthComplete(configs)  # noqa

    @staticmethod
    def get_rotation_for_obj(base_rt, xyz, degrees=15):
        xy = xyz[1:3]
        fake_x = 1
        fake_y = xy[0] * fake_x / xy[1]
        start_theta = math.atan2(fake_y, fake_x) + math.pi
        logger.info("degrees: {}".format(math.degrees(start_theta)))
        radians = math.radians(degrees)
        # generate pose
        count = 1
        pose_list = list()
        while True:
            current_radians = start_theta + count * radians
            if start_theta + math.pi > current_radians:
                pose_rotate_euler = [current_radians, 0, 0]
                R = Rotation.from_euler("xyz", pose_rotate_euler, degrees=False)  # noqa
                R_3x3 = R.as_matrix()  # noqa
                base_pose = np.eye(4)  # noqa
                base_pose[0:3, 0:3] = R_3x3
                pose = np.dot(base_pose, base_rt)
                base_pose[0: 3, 3] = xyz
                base_pose[0: 3, 0: 3] = pose[0: 3, 0: 3]
                pose_list.append(base_pose)
                count += 1
            else:
                break

        return pose_list

    @staticmethod
    def add_bias_to_xyz(pose, bias):
        xyz = pose[0:3, 3]
        xy = xyz[1:3]
        xy_norm = np.linalg.norm(xy)

        theta = math.atan2(xy[0], xy[1])
        print(xy)
        xy[1] = math.cos(theta) * (xy_norm + bias)
        xy[0] = math.sin(theta) * (xy_norm + bias)
        print(xy)
        print(xy_norm, np.linalg.norm(xy))
        pose[1:3, 3] = xy
        return pose
