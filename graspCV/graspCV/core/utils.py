import tensorrt as trt


def dtype_to_size(dtype: trt.DataType):
    data_type_to_size = {
        trt.DataType.FLOAT: 4,
        trt.DataType.HALF: 2,
        trt.DataType.INT32: 4,
        trt.DataType.INT8: 1,
        trt.DataType.BOOL: 1
    }
    return data_type_to_size.get(dtype, 4)


def dims_to_size(dims):
    size = 1
    for d in dims:
        size *= d
    return size
