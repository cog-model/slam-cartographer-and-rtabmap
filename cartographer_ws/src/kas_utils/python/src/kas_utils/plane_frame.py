import numpy as np


class PlaneFrame():
    def __init__(self):
        self.T = None
        self.invT = None

    @classmethod
    def from_plane_frame_pose(cls, T):
        obj = cls()
        obj.set(T)
        return obj

    @classmethod
    def from_points(cls, points):
        obj = cls()
        obj.set_from_points(points)
        return obj

    def set(self, T):
        assert isinstance(T, np.ndarray)
        assert T.shape == (4, 4)
        self.T = T
        self.invT = np.linalg.inv(T)

    def is_set(self):
        return self.T is not None

    def origin2plane(self):
        return self.T

    def plane2origin(self):
        return self.invT

    def distance_from_origin_to_plane(self, shift=0):
        assert self.is_set()
        dist = -self.invT[2, 3] + shift
        return dist

    def orthogonal_distance_from_origin(self, points):
        assert self.is_set()
        dist = np.dot(points, self.T[:3, 2])
        return dist

    def distance_to_plane(self, points, shift=0):
        assert self.is_set()
        dist = self.distance_from_origin_to_plane(shift=shift) - \
            self.orthogonal_distance_from_origin(points)
        return dist

    def to_plane(self, data_in_origin, is_poses=False, shift=0):
        assert self.is_set()
        data_in_origin, is_padded = \
            self._prepare_data_for_processing(data_in_origin, is_poses)
        shifted_invT = self.invT.copy()
        shifted_invT[2, 3] -= shift
        data_in_plane = np.matmul(shifted_invT, data_in_origin)
        data_in_plane = \
            self._prepare_data_for_return(data_in_plane, is_poses, is_padded)
        return data_in_plane

    def to_origin(self, data_in_plane, is_poses=False, shift=0):
        assert self.is_set()
        data_in_plane, is_padded = \
            self._prepare_data_for_processing(data_in_plane, is_poses)
        shifted_T = self.T.copy()
        shifted_T[:3, 3] += self.T[:3, 2] * shift
        data_in_origin = np.matmul(shifted_T, data_in_plane)
        data_in_origin = \
            self._prepare_data_for_return(data_in_origin, is_poses, is_padded)
        return data_in_origin

    def project_points(self, points, shift=0):
        assert self.is_set()
        dist = self.distance_to_plane(points, shift=shift)
        dist = np.expand_dims(dist, axis=-1)
        direction = self.T[:3, 2]
        points = points + direction * dist
        return points

    def intersection_with_plane(self, points, shift=0):
        assert self.is_set()
        k = self.distance_from_origin_to_plane(shift=shift) / \
            self.orthogonal_distance_from_origin(points)
        k = np.expand_dims(k, axis=-1)
        points = k * points
        return points

    def set_from_points(self, points):
        # ax + by + c = z

        # A * plane = B
        # plane = [a, b, c]
        # A = [[xi, yi, 1]]
        # B = [zi]

        # plane = inv(AT * A) * AT * B

        n = points.shape[0]
        assert n % 2 == 0

        A = np.hstack((points[:, 0:2], np.ones((n, 1))))
        B = points[:, 2]
        plane = np.matmul(np.matmul(np.linalg.inv(np.matmul(A.T, A)), A.T), B)

        centroid = np.sum(points, axis=0) / n
        origin = self._project_to_plane(centroid, plane)

        a = plane[0]
        b = plane[1]
        c = plane[2]
        z_axis = np.array([-a, -b, 1])
        z_axis /= np.linalg.norm(z_axis)

        x_direction = \
            (np.sum(points[:int(n/2)], axis=0) / (n/2) +
            2 * centroid - np.sum(points[int(n/2):], axis=0) / (n/2)) / 2
        x_axis = self._project_to_plane(x_direction, plane) - origin
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        if np.dot(z_axis, origin) > 0:
            y_axis *= -1
            z_axis *= -1

        x_axis = np.expand_dims(x_axis, axis=-1)
        y_axis = np.expand_dims(y_axis, axis=-1)
        z_axis = np.expand_dims(z_axis, axis=-1)

        R = np.hstack((x_axis, y_axis, z_axis))
        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0:3, 3] = origin
        self.set(T)

    @staticmethod
    def _project_to_plane(p0, plane):
        x0 = p0[0]
        y0 = p0[1]
        z0 = p0[2]
        a = plane[0]
        b = plane[1]
        c = plane[2]

        dz = (a * x0 + b * y0 + c - z0) / (a * a + b * b + 1)
        x = x0 - a * dz
        y = y0 - b * dz
        z = z0 + dz
        return np.array([x, y, z])

    @staticmethod
    def _prepare_data_for_processing(data, is_poses):
        if not is_poses:
            assert data.shape[-1] in (3, 4)
        else:
            assert data.shape[-2:] == (4, 4)

        if not is_poses:
            if data.shape[-1] == 3:
                pad_width = [(0, 0)] * data.ndim
                pad_width[-1] = (0, 1)
                data = np.pad(data, pad_width, mode='constant', constant_values=1)
                is_padded = True
            elif data.shape[-1] == 4:
                is_padded = False
            data = np.expand_dims(data, axis=-1)
            # data.shape = (..., 4, 1)
        else:
            # data.shape = (..., 4, 4)
            is_padded = None

        return data, is_padded

    @staticmethod
    def _prepare_data_for_return(data, is_poses, is_padded):
        if not is_poses:
            data = data[..., 0]
            if is_padded:
                data = data[..., :3]
        return data

