/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_POSE_GRAPH_3D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_POSE_GRAPH_3D_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer/mapping/internal/constraints/constraint_builder_3d.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_3d.h"
#include "cartographer/mapping/internal/pose_graph_data.h"
#include "cartographer/mapping/internal/pose_graph_constraints.h"
#include "cartographer/mapping/internal/pose_graph_trajectory_states.h"
#include "cartographer/mapping/internal/pose_graph_maps.h"
#include "cartographer/mapping/internal/work_queue.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"

#include "kas_utils/time_measurer.h"

namespace cartographer {
namespace mapping {

class PoseGraph3D : public PoseGraph {
public:
  PoseGraph3D(
      const proto::PoseGraphOptions& options,
      std::unique_ptr<optimization::OptimizationProblem3D> optimization_problem,
      common::ThreadPool* thread_pool);
  ~PoseGraph3D() override;

  PoseGraph3D(const PoseGraph3D&) = delete;
  PoseGraph3D& operator=(const PoseGraph3D&) = delete;

  NodeId AddNode(
      std::shared_ptr<const TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap3D>>& insertion_submaps)
          ABSL_LOCKS_EXCLUDED(mutex_)
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);

  void AddImuData(
      int trajectory_id,
      const sensor::ImuData& imu_data) override
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void AddOdometryData(
      int trajectory_id,
      const sensor::OdometryData& odometry_data) override
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void AddFixedFramePoseData(
      int trajectory_id,
      const sensor::FixedFramePoseData& fixed_frame_pose_data) override
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void AddLandmarkData(
      int trajectory_id,
      const sensor::LandmarkData& landmark_data) override
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);

  void DeleteTrajectory(int trajectory_id) override
      ABSL_LOCKS_EXCLUDED(mutex_)
      ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void FinishTrajectory(int trajectory_id) override
      ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  bool IsTrajectoryFinished(int trajectory_id) const override
      ABSL_LOCKS_EXCLUDED(mutex_);
  void FreezeTrajectory(int trajectory_id) override
      ABSL_LOCKS_EXCLUDED(mutex_)
      ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  bool IsTrajectoryFrozen(int trajectory_id) const override
      ABSL_LOCKS_EXCLUDED(mutex_);

  void ScheduleNodesToTrim(const std::set<common::Time>& nodes_to_trim) override
      ABSL_LOCKS_EXCLUDED(mutex_);
  void ScheduleNodesToTrim(const std::set<NodeId>& nodes_to_trim) override
      ABSL_LOCKS_EXCLUDED(mutex_);

  void AddSubmapFromProto(
      const transform::Rigid3d& global_submap_pose,
      const proto::Submap& submap) override
          ABSL_LOCKS_EXCLUDED(mutex_)
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void AddNodeFromProto(
      const transform::Rigid3d& global_pose,
      const proto::Node& node) override
          ABSL_LOCKS_EXCLUDED(mutex_)
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void SetTrajectoryDataFromProto(
      const proto::TrajectoryData& data) override
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void SetLandmarkPose(
      const std::string& landmark_id,
      const transform::Rigid3d& global_pose,
      bool frozen = false) override
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void AddSerializedConstraints(
      const std::vector<Constraint>& constraints) override
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void AddSerializedMaps(
      const std::map<std::string, std::set<int>>& maps_data) override
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);

  void AddPureLocalizationTrimmer(int trajectory_id,
      const proto::PureLocalizationTrimmerOptions& pure_localization_trimmer_options) override
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void AddLoopTrimmer(
      int trajectory_id,
      const proto::LoopTrimmerOptions& loop_trimmer_options) override
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);

  void ScheduleFalseConstraintsTrimming(
      double max_rotation_error, double max_translation_error) override
          ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void RunFinalOptimization() override
      ABSL_LOCKS_EXCLUDED(mutex_)
      ABSL_LOCKS_EXCLUDED(work_queue_mutex_);

  std::vector<std::vector<int>> GetConnectedTrajectories() const override
      ABSL_LOCKS_EXCLUDED(mutex_);
  bool TrajectoriesTransitivelyConnected(
      int trajectory_a, int trajectory_b) const override
          ABSL_LOCKS_EXCLUDED(mutex_);
  common::Time TrajectoriesLastConnectionTime(
      int trajectory_a, int trajectory_b) const override
          ABSL_LOCKS_EXCLUDED(mutex_);
  bool TrajectoriesBelongToTheSameMap(
      int trajectory_a, int trajectory_b) const override
          ABSL_LOCKS_EXCLUDED(mutex_);

  transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) const
      ABSL_LOCKS_EXCLUDED(mutex_);

  void MoveTrajectoryToMap(int trajectory_id, const std::string& map_name) override
      ABSL_LOCKS_EXCLUDED(work_queue_mutex_);

  PoseGraph::SubmapData GetSubmapData(const SubmapId& submap_id) const
      ABSL_LOCKS_EXCLUDED(mutex_);
  MapById<SubmapId, SubmapData> GetAllSubmapData() const
      ABSL_LOCKS_EXCLUDED(mutex_);
  MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const
      ABSL_LOCKS_EXCLUDED(mutex_);
  MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const override
      ABSL_LOCKS_EXCLUDED(mutex_);
  MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const override
      ABSL_LOCKS_EXCLUDED(mutex_);
  std::map<int, TrajectoryState> GetTrajectoryStates() const override
      ABSL_LOCKS_EXCLUDED(mutex_);
  std::map<std::string, transform::Rigid3d> GetLandmarkPoses() const override
      ABSL_LOCKS_EXCLUDED(mutex_);
  sensor::MapByTime<sensor::ImuData> GetImuData() const override
      ABSL_LOCKS_EXCLUDED(mutex_);
  sensor::MapByTime<sensor::OdometryData> GetOdometryData() const override
      ABSL_LOCKS_EXCLUDED(mutex_);
  sensor::MapByTime<sensor::FixedFramePoseData> GetFixedFramePoseData() const override
      ABSL_LOCKS_EXCLUDED(mutex_);
  std::map<std::string, PoseGraph::LandmarkNode> GetLandmarkNodes() const override
      ABSL_LOCKS_EXCLUDED(mutex_);
  std::map<int, TrajectoryData> GetTrajectoryData() const override
      ABSL_LOCKS_EXCLUDED(mutex_);
  std::map<std::string, std::set<int>> GetMapsData() const override
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  std::vector<Constraint> constraints() const override
      ABSL_LOCKS_EXCLUDED(mutex_);

  void SetInitialTrajectoryPose(
      int from_trajectory_id, int to_trajectory_id,
      const transform::Rigid3d& pose, common::Time time) override
          ABSL_LOCKS_EXCLUDED(mutex_);

  void SetGlobalSlamOptimizationCallback(
      PoseGraphInterface::GlobalSlamOptimizationCallback callback) override
          ABSL_LOCKS_EXCLUDED(mutex_);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

  void WaitForAllComputations()
      ABSL_LOCKS_EXCLUDED(mutex_)
      ABSL_LOCKS_EXCLUDED(work_queue_mutex_);

  void WaitForQueue() override
      ABSL_LOCKS_EXCLUDED(work_queue_mutex_);

private:
  void AddWorkItem(const std::function<WorkItem::Result()>& work_item)
      ABSL_LOCKS_EXCLUDED(work_queue_mutex_);
  void HandleWorkQueue(const constraints::ConstraintBuilder3D::Result& result)
      ABSL_LOCKS_EXCLUDED(mutex_)
      ABSL_LOCKS_EXCLUDED(work_queue_mutex_)
      ABSL_LOCKS_EXCLUDED(executing_work_item_mutex_);
  void DrainWorkQueue()
      ABSL_LOCKS_EXCLUDED(mutex_)
      ABSL_LOCKS_EXCLUDED(work_queue_mutex_)
      ABSL_LOCKS_EXCLUDED(executing_work_item_mutex_);

  void RunOptimization()
      ABSL_LOCKS_EXCLUDED(mutex_)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  void TrimSubmap(const SubmapId& submap_id)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);
  void ReattachLoop(Constraint& loop, const NodeId& to_node_id)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);
  void ReattachLoop(Constraint& loop, const SubmapId& to_submap_id)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);
  void TrimNode(const NodeId& node_id)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  std::vector<Constraint> TrimFalseDetectedLoops(
      const std::vector<Constraint>& new_loops)
          ABSL_LOCKS_EXCLUDED(mutex_)
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);
  void TrimLoopsInWindow()
      ABSL_LOCKS_EXCLUDED(mutex_)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);
  std::vector<Constraint> TrimLoops(
      const std::vector<Constraint>& new_loops)
          ABSL_LOCKS_EXCLUDED(mutex_)
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  void TrimPureLocalizationTrajectories()
      ABSL_LOCKS_EXCLUDED(mutex_)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);
  void TrimScheduledNodes()
      ABSL_LOCKS_EXCLUDED(mutex_)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  std::pair<bool, bool> CheckIfConstraintCanBeAdded(
      const NodeId& node_id, const SubmapId& submap_id)
          ABSL_LOCKS_EXCLUDED(mutex_)
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  std::pair<std::vector<SubmapId>, std::vector<SubmapId>>
      ComputeCandidatesForConstraints(const NodeId& node_id)
          ABSL_LOCKS_EXCLUDED(mutex_)
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);
  std::pair<std::vector<NodeId>, std::vector<NodeId>>
      ComputeCandidatesForConstraints(const SubmapId& submap_id)
          ABSL_LOCKS_EXCLUDED(mutex_)
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  static std::vector<SubmapId>
      SelectCandidatesForConstraints(
          const std::vector<SubmapId>& candidates,
          double& num_constraints_to_compute,
          std::set<SubmapId>& submaps_used_for_constraints)
              ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);
  std::pair<std::vector<SubmapId>, std::vector<SubmapId>>
      SelectCandidatesForConstraints(
          const std::vector<SubmapId>& local_candidates,
          const std::vector<SubmapId>& global_candidates)
              ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  static std::vector<NodeId>
      SelectCandidatesForConstraints(
          const std::vector<NodeId>& candidates,
          double& num_constraints_to_compute)
              ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);
  std::pair<std::vector<NodeId>, std::vector<NodeId>>
      SelectCandidatesForConstraints(
          const std::vector<NodeId>& local_candidates,
          const std::vector<NodeId>& global_candidates)
              ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  void MaybeAddConstraints(const NodeId& node_id,
      const std::vector<SubmapId>& local_submap_ids,
      const std::vector<SubmapId>& global_submap_ids)
          ABSL_LOCKS_EXCLUDED(mutex_)
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);
  void MaybeAddConstraints(const SubmapId& submap_id,
      const std::vector<NodeId>& local_node_ids,
      const std::vector<NodeId>& global_node_ids)
          ABSL_LOCKS_EXCLUDED(mutex_)
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  std::vector<SubmapId> InitializeGlobalSubmapPoses(
      int trajectory_id, const common::Time time,
      const std::vector<std::shared_ptr<const Submap3D>>& insertion_submaps)
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_)
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  WorkItem::Result ComputeConstraintsForNode(
      const NodeId& node_id,
      std::vector<std::shared_ptr<const Submap3D>> insertion_submaps,
      bool newly_finished_submap)
          ABSL_LOCKS_EXCLUDED(mutex_)
          ABSL_LOCKS_EXCLUDED(executing_work_item_mutex_);

  NodeId AppendNode(
      std::shared_ptr<const TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap3D>>& insertion_submaps,
      const transform::Rigid3d& optimized_pose)
          ABSL_LOCKS_EXCLUDED(mutex_);

  transform::Rigid3d GetInterpolatedGlobalTrajectoryPose(
      int trajectory_id, const common::Time time) const
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  transform::Rigid3d ComputeLocalToGlobalTransform(
      const MapById<SubmapId, optimization::SubmapSpec3D>& global_submap_poses,
      int trajectory_id) const
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  void DeleteTrajectoriesIfNeeded()
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  PoseGraph::SubmapData GetSubmapDataUnderLock(const SubmapId& submap_id) const
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  MapById<SubmapId, SubmapData> GetSubmapDataUnderLock() const
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  common::Time GetLatestNodeTime(
      const NodeId& node_id, const SubmapId& submap_id) const
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_)
          ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);
  
  void UpdateTrajectoryConnectivity(const Constraint& constraint)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

  void LogResidualHistograms() const
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(mutex_)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(executing_work_item_mutex_);

private:
  mutable absl::Mutex mutex_;
  absl::Mutex work_queue_mutex_;
  absl::Mutex executing_work_item_mutex_;

  const proto::PoseGraphOptions options_;
  std::unique_ptr<optimization::OptimizationProblem3D> optimization_problem_;
  constraints::ConstraintBuilder3D constraint_builder_;
  common::ThreadPool* const thread_pool_;

  std::unique_ptr<WorkQueue> work_queue_ ABSL_GUARDED_BY(work_queue_mutex_);

  int num_nodes_since_last_loop_closure_ ABSL_GUARDED_BY(executing_work_item_mutex_);

  std::map<int, proto::PureLocalizationTrimmerOptions> pure_localization_trimmer_options_
      ABSL_GUARDED_BY(executing_work_item_mutex_);
  std::map<int, proto::LoopTrimmerOptions> loop_trimmer_options_
      ABSL_GUARDED_BY(executing_work_item_mutex_);
  std::set<int> pure_localization_trajectory_ids_
      ABSL_GUARDED_BY(executing_work_item_mutex_);

  double num_local_constraints_to_compute_ ABSL_GUARDED_BY(executing_work_item_mutex_);
  double num_global_constraints_to_compute_ ABSL_GUARDED_BY(executing_work_item_mutex_);
  std::set<SubmapId> submaps_used_for_local_constraints_
      ABSL_GUARDED_BY(executing_work_item_mutex_);
  std::set<SubmapId> submaps_used_for_global_constraints_
      ABSL_GUARDED_BY(executing_work_item_mutex_);

  std::set<NodeId> nodes_scheduled_to_trim_ ABSL_GUARDED_BY(mutex_);

  PoseGraphData data_ ABSL_GUARDED_BY(mutex_);
  PoseGraphConstraints constraints_ ABSL_GUARDED_BY(mutex_);
  PoseGraphTrajectoryStates trajectory_states_ ABSL_GUARDED_BY(mutex_);
  PoseGraphMaps maps_ ABSL_GUARDED_BY(mutex_);

  GlobalSlamOptimizationCallback global_slam_optimization_callback_
      ABSL_GUARDED_BY(executing_work_item_mutex_);
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_POSE_GRAPH_3D_H_
