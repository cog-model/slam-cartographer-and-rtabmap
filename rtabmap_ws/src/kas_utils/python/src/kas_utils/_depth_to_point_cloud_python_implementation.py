import numpy as np
import torch
from .utils import get_depth_scale


class DepthToPointCloud:
    def __init__(self, fx, fy, cx, cy, pool_size):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.pool_size = pool_size

    def min_pool(self, input):
        orig_num_threads = torch.get_num_threads()
        torch.set_num_threads(1)

        if input.ndim == 2:
            input = np.expand_dims(input, axis=0)
            expanded = True
        else:
            expanded = False

        max_pool_fn = torch.nn.MaxPool2d(self.pool_size)
        pooled = -max_pool_fn(-torch.from_numpy(input)).numpy()

        if expanded:
            pooled = pooled[0]

        torch.set_num_threads(orig_num_threads)
        return pooled

    def convert(self, depth):
        if self.pool_size > 1:
            scale = get_depth_scale(depth)
            depth = depth * scale
            invalid = (depth <= 0) | ~np.isfinite(depth)
            depth[invalid] = np.inf
            depth = self.min_pool(depth)
            fx = self.fx / self.pool_size
            fy = self.fy / self.pool_size
            cx = self.cx / self.pool_size
            cy = self.cy / self.pool_size
            valid = np.isfinite(depth)
            z = depth[valid]
            v, u = np.where(valid)
            x = (u - cx) / fx * z
            y = (v - cy) / fy * z
        else:
            scale = get_depth_scale(depth)
            valid = (depth > 0) & np.isfinite(depth)
            fx = self.fx
            fy = self.fy
            cx = self.cx
            cy = self.cy
            z = depth[valid]
            z = z * scale
            v, u = np.where(valid)
            x = (u - cx) / fx * z
            y = (v - cy) / fy * z

        point_cloud = np.vstack((x, y, z)).transpose()

        if point_cloud.dtype != np.float32:
            point_cloud = point_cloud.astype(np.float32)
        if not point_cloud.flags['C_CONTIGUOUS']:
            point_cloud = np.ascontiguousarray(point_cloud)
        return point_cloud

    def set_camera_intrinsics(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def set_pool_size(self, pool_size):
        self.pool_size = pool_size
