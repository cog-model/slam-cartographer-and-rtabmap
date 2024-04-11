-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "local_map_lidar",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  publish_to_tf = true,
  publish_tracked_pose = true,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  optimization_results_only_connected_trajectories = true,
  optimization_results_only_recently_connected_trajectories = false,
  log_trajectories_connection_time = true,
}

MAP_BUILDER.use_trajectory_builder_3d = true
TRAJECTORY_BUILDER_3D.use_imu_data = false
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- Range filter --
TRAJECTORY_BUILDER_3D.min_range = 1.
MAX_3D_RANGE = 120.
TRAJECTORY_BUILDER_3D.max_range = MAX_3D_RANGE

-- Voxel filter --
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 15.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = MAX_3D_RANGE

-- Motion filter --
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(20)

-- Submaps --
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 20
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.1
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45

-- Local SLAM --
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 10

-- Global SLAM --
MAP_BUILDER.num_background_threads = 1
POSE_GRAPH.optimize_every_n_nodes = 0

-- Constraint search --
POSE_GRAPH.max_local_constraint_distance = 15.
POSE_GRAPH.local_constraints_per_node = 0
POSE_GRAPH.global_constraints_per_node = 0
POSE_GRAPH.constraint_builder.min_score = 0.40
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.40
POSE_GRAPH.global_constraint_search_after_n_seconds = 0.

-- Optimization problem --
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.optimization_problem.add_local_slam_consecutive_node_constraints_in_3d = true
POSE_GRAPH.optimization_problem.add_odometry_consecutive_node_constraints_in_3d = false

-- Logs --
TRAJECTORY_BUILDER.log_data_frequency = false
TRAJECTORY_BUILDER_3D.motion_filter.log_number_of_nodes_after_reduction = false
POSE_GRAPH.log_work_queue_size = true
POSE_GRAPH.log_constraints = true
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.log_number_of_trimmed_loops = true
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.optimization_problem.log_solver_summary = false

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

return options
