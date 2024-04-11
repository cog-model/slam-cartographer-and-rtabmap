import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation


class BaseOdometry:
    POINT_TO_POINT = 0
    POINT_TO_PLANE = 1

    def __init__(self, voxel_size, max_correspondence_distances, icp_method):
        assert icp_method in (
            BaseOdometry.POINT_TO_POINT,
            BaseOdometry.POINT_TO_PLANE)

        self.voxel_size = voxel_size
        self.max_correspondence_distances = max_correspondence_distances
        self.icp_method = icp_method

    def _register(self, source, target, init_relative_pose):
        if self.icp_method == BaseOdometry.POINT_TO_POINT:
            estimation_method = \
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
        elif self.icp_method == BaseOdometry.POINT_TO_PLANE:
            estimation_method = \
                o3d.pipelines.registration.TransformationEstimationPointToPlane()

        relative_pose = init_relative_pose
        for max_correspondence_distance in self.max_correspondence_distances:
            reg = o3d.pipelines.registration.registration_icp(
                source, target,
                max_correspondence_distance, init=relative_pose,
                estimation_method=estimation_method)
            relative_pose = reg.transformation
            if relative_pose is None:
                raise RuntimeError("ICP failed.")
        return relative_pose

    def _estimate_normals(self, pc):
        assert not pc.has_normals()
        pc.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 4, max_nn=30),
            fast_normal_computation=False)

    def compute(self, pc: o3d.geometry.PointCloud):
        raise NotImplementedError()


class Odometry(BaseOdometry):
    def __init__(self, voxel_size, max_correspondence_distances, icp_method):
        super().__init__(voxel_size, max_correspondence_distances, icp_method)

        self.last_pose = np.eye(4)
        self.last_frame = None

    def compute(self, pc: o3d.geometry.PointCloud):
        if self.last_frame is None:
            self.last_frame = pc.voxel_down_sample(self.voxel_size)
            if self.icp_method == BaseOdometry.POINT_TO_PLANE and \
                    not self.last_frame.has_normals():
                self._estimate_normals(self.last_frame)
            return self.last_pose

        down_pc = pc.voxel_down_sample(self.voxel_size)
        relative_pose = self._register(down_pc, self.last_frame, np.eye(4))
        pose = np.matmul(self.last_pose, relative_pose)

        self.last_pose = pose
        self.last_frame = down_pc
        if self.icp_method == BaseOdometry.POINT_TO_PLANE and \
                not self.last_frame.has_normals():
            self._estimate_normals(self.last_frame)
        return self.last_pose


class ReferenceFrameOdometry(BaseOdometry):
    def __init__(self, voxel_size, max_correspondence_distances, icp_method,
            fitness_threshold=0.8, store_reference_frames=False):
        super().__init__(voxel_size, max_correspondence_distances, icp_method)
        self.fitness_threshold = fitness_threshold
        self.store_reference_frames = store_reference_frames

        self.last_pose = np.eye(4)

        self.reference_pose = None
        self.reference_frame = None
        self.reference_frame_changed = False
        self.reference_frames_counter = 0

        if self.store_reference_frames:
            self.reference_poses = list()
            self.reference_frames = list()
        else:
            self.reference_poses = None
            self.reference_frames = None

    def compute(self, pc: o3d.geometry.PointCloud):
        if self.reference_frame is None:
            self.reference_pose = self.last_pose
            self.reference_frame = pc.voxel_down_sample(self.voxel_size)
            if self.icp_method == BaseOdometry.POINT_TO_PLANE and \
                    not self.reference_frame.has_normals():
                self._estimate_normals(self.reference_frame)
            self.reference_frame_changed = True
            self.reference_frames_counter += 1

            if self.store_reference_frames:
                self.reference_poses.append(self.reference_pose)
                self.reference_frames.append(self.reference_frame)
            return self.last_pose

        down_pc = pc.voxel_down_sample(self.voxel_size)
        relative_pose = np.matmul(np.linalg.inv(self.reference_pose), self.last_pose)
        relative_pose = self._register(down_pc, self.reference_frame, relative_pose)
        pose = np.matmul(self.reference_pose, relative_pose)

        results = o3d.pipelines.registration.evaluate_registration(
            down_pc, self.reference_frame,
            self.voxel_size * 1.5, transformation=relative_pose)
        if results.fitness < self.fitness_threshold:
            self.reference_pose = pose.copy()  # copy() is needed because
                                               # "pose" is returned from this function
            self.reference_frame = down_pc
            if self.icp_method == BaseOdometry.POINT_TO_PLANE and \
                    not self.reference_frame.has_normals():
                self._estimate_normals(self.reference_frame)
            self.reference_frame_changed = True
            self.reference_frames_counter += 1

            if self.store_reference_frames:
                self.reference_poses.append(self.reference_pose)
                self.reference_frames.append(self.reference_frame)
        else:
            self.reference_frame_changed = False

        self.last_pose = pose
        return self.last_pose
    

class PoseGraph:
    POINT_TO_POINT = 0
    POINT_TO_PLANE = 1

    def __init__(self, reference_poses, reference_frames,
            max_correspondence_distances, voxel_size, refine_icp_method,
            fitness_threshold=0.6,
            max_position_correction_rate=0.05, max_orientation_correction_rate=0.03):
        assert len(reference_poses) == len(reference_frames)
        assert len(reference_poses) >= 3
        assert refine_icp_method in (
            PoseGraph.POINT_TO_POINT,
            PoseGraph.POINT_TO_PLANE)

        self.reference_poses = reference_poses
        self.reference_frames = reference_frames
        self.max_correspondence_distances = max_correspondence_distances
        self.voxel_size = voxel_size
        self.refine_icp_method = refine_icp_method
        self.fitness_threshold = fitness_threshold
        self.max_position_correction_rate = max_position_correction_rate
        self.max_orientation_correction_rate = max_orientation_correction_rate

        self.pose_graph = o3d.pipelines.registration.PoseGraph()
        self._fill_pose_graph()

        self.accum_distances = list()
        self.accum_angles = list()
        self._fill_accum()

        self._loops = set()

    def _fill_pose_graph(self):
        for pose in self.reference_poses:
            self.pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(pose))

        for i, (prev_pose, prev_frame, pose, frame) in \
                enumerate(zip(
                    self.reference_poses[:-1], self.reference_frames[:-1],
                    self.reference_poses[1:], self.reference_frames[1:])):
            relative_pose_inv = np.matmul(np.linalg.inv(pose), prev_pose)
            information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                prev_frame, frame, self.voxel_size * 1.5, relative_pose_inv)
            self.pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
                i,
                i + 1,
                relative_pose_inv,
                information,
                uncertain=False))
            
    def _fill_accum(self):
        accum_distance = 0
        accum_angle = 0
        self.accum_distances.append(accum_distance)
        self.accum_angles.append(accum_angle)
        for prev_pose, pose in zip(self.reference_poses[:-1], self.reference_poses[1:]):
            relative_pose = np.matmul(np.linalg.inv(prev_pose), pose)
            translation = relative_pose[:3, 3]
            rotation = relative_pose[:3, :3]
            distance = np.linalg.norm(translation)
            angle = np.linalg.norm(Rotation.from_matrix(rotation).as_rotvec())

            accum_distance += distance
            accum_angle += angle
            self.accum_distances.append(accum_distance)
            self.accum_angles.append(accum_angle)

    def _register(self, source, target, init_relative_pose):
        if self.refine_icp_method == PoseGraph.POINT_TO_POINT:
            estimation_method = \
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
        elif self.refine_icp_method == PoseGraph.POINT_TO_PLANE:
            estimation_method = \
                o3d.pipelines.registration.TransformationEstimationPointToPlane()

        relative_pose = init_relative_pose
        for max_correspondence_distance in self.max_correspondence_distances:
            reg = o3d.pipelines.registration.registration_icp(
                source, target,
                max_correspondence_distance, init=relative_pose,
                estimation_method=estimation_method)
            relative_pose = reg.transformation
            if relative_pose is None:
                raise RuntimeError("ICP failed.")
        return relative_pose

    def compute_loops(self):
        for i, (node_i, frame_i) in \
                enumerate(zip(self.pose_graph.nodes, self.reference_frames)):

            for j, (node_j, frame_j) in \
                    enumerate(zip(self.pose_graph.nodes[i + 2:], self.reference_frames[i + 2:]),
                        start=(i + 2)):

                if (i, j) in self._loops:
                    continue

                pose_i = node_i.pose
                pose_j = node_j.pose
                relative_pose_inv = np.matmul(np.linalg.inv(pose_j), pose_i)
                relative_pose_inv_corrected = self._register(
                    frame_i, frame_j, relative_pose_inv)

                results = o3d.pipelines.registration.evaluate_registration(
                    frame_i, frame_j, self.voxel_size * 1.5,
                    transformation=relative_pose_inv_corrected)
                if results.fitness < self.fitness_threshold:
                    continue

                correction = np.matmul(
                    np.linalg.inv(relative_pose_inv), relative_pose_inv_corrected)
                correction_translation = correction[:3, 3]
                correction_rotation = correction[:3, :3]
                position_correction = np.linalg.norm(correction_translation)
                orientation_correction = np.linalg.norm(Rotation.from_matrix(correction_rotation).as_rotvec())

                distance = self.accum_distances[j] - self.accum_distances[i]
                angle = self.accum_angles[j] - self.accum_angles[i]

                position_correction_rate = position_correction / distance
                orientation_correction_rate = orientation_correction / angle
                if position_correction_rate > self.max_position_correction_rate or \
                        orientation_correction_rate > self.max_orientation_correction_rate:
                    continue

                information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                    frame_i, frame_j, self.voxel_size * 1.5, relative_pose_inv_corrected)
                self.pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
                    i,
                    j,
                    relative_pose_inv_corrected,
                    information,
                    uncertain=True))
                self._loops.add((i, j))

    def optimize(self):
        o3d.pipelines.registration.global_optimization(
            self.pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            o3d.pipelines.registration.GlobalOptimizationOption(
                max_correspondence_distance=self.voxel_size * 1.5,
                edge_prune_threshold=0.25,
                reference_node=0))

    @staticmethod
    def _lineset_to_cylinders(lineset):
        cylinders = list()
        points = np.asarray(lineset.points)
        for line, color in zip(np.asarray(lineset.lines), np.asarray(lineset.colors)):
            p1 = points[line[0]]
            p2 = points[line[1]]
            length = np.linalg.norm(p2 - p1)
            if length < 0.001:
                continue

            z = (p2 - p1) / length

            min_index = np.argmin(np.abs(z))
            other_indices = np.delete(np.array([0, 1, 2]), min_index)
            x = np.zeros((3,))
            x[other_indices] = z[other_indices[::-1]] / np.linalg.norm(z[other_indices])
            x[other_indices[0]] *= -1

            y = np.cross(z, x)

            center = (p1 + p2) / 2

            transform = np.eye(4)
            transform[:3, :3] = np.stack((x, y, z), axis=1)
            transform[:3, 3] = center

            cylinder = o3d.geometry.TriangleMesh.create_cylinder(
                radius=0.001, height=length)
            cylinder.transform(transform)
            cylinder.paint_uniform_color(color)
            cylinders.append(cylinder)
        return cylinders

    def visualize(self):
        frames_axes = list()
        connections_lines = o3d.geometry.LineSet()
        corrections_lines = o3d.geometry.LineSet()
        for node in self.pose_graph.nodes:
            pose = node.pose
            frame_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            frame_axes.transform(pose)
            frames_axes.append(frame_axes)

            position = pose[:3, 3]
            connections_lines.points.append(position)
            corrections_lines.points.append(position)

        for edge in self.pose_graph.edges:
            connections_lines.lines.append([edge.source_node_id, edge.target_node_id])
            if edge.source_node_id + 1 == edge.target_node_id:
                # odometry
                connections_lines.colors.append([1.0, 0.0, 0.0])
            else:
                # loop
                connections_lines.colors.append([1.0, 1.0, 0.0])
        
        for i, pose in enumerate(self.reference_poses):
            position = pose[:3, 3]
            corrections_lines.points.append(position)
            corrections_lines.lines.append([i, i + len(self.reference_poses)])
            corrections_lines.colors.append([0.0, 0.0, 1.0])

        cylinders = list()
        cylinders.extend(self._lineset_to_cylinders(connections_lines))
        cylinders.extend(self._lineset_to_cylinders(corrections_lines))

        merged_pc = o3d.geometry.PointCloud()
        for node, frame in zip(self.pose_graph.nodes, self.reference_frames):
            pc = o3d.geometry.PointCloud(frame)
            pc.transform(node.pose)
            merged_pc += pc
        merged_pc = merged_pc.voxel_down_sample(0.005)

        o3d.visualization.draw_geometries(frames_axes + cylinders + [merged_pc])
