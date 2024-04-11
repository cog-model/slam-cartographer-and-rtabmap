import os

try:
    from .py_depth_to_point_cloud import DepthToPointCloud
except ModuleNotFoundError:
    if int(os.environ.get('DEPTH_TO_POINT_CLOUD_ALLOW_PYTHON_IMPLEMENTATION', 0)):
        from ._depth_to_point_cloud_python_implementation import DepthToPointCloud
        os.environ['DEPTH_TO_POINT_CLOUD_PYTHON_IMPLEMENTATION'] = '1'
    else:
        raise ImportError(
            "Seems like you did not compile depth_to_point_cloud c++ module. "
            "Try to use DEPTH_TO_POINT_CLOUD_ALLOW_PYTHON_IMPLEMENTATION=1 environmental "
            "variable to allow python implementation or compile c++ module.")
else:
    os.environ['DEPTH_TO_POINT_CLOUD_PYTHON_IMPLEMENTATION'] = '0'
