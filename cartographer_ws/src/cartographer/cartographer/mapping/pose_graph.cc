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

#include "cartographer/mapping/pose_graph.h"

#include "cartographer/mapping/internal/constraints/constraint_builder.h"
#include "cartographer/mapping/internal/optimization/optimization_problem_options.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

void PopulateOverlappingSubmapsTrimmerOptions2D(
    proto::PoseGraphOptions* const pose_graph_options,
    common::LuaParameterDictionary* const parameter_dictionary) {
  constexpr char kDictionaryKey[] = "overlapping_submaps_trimmer_2d";
  if (!parameter_dictionary->HasKey(kDictionaryKey)) return;

  auto options_dictionary = parameter_dictionary->GetDictionary(kDictionaryKey);
  auto* options = pose_graph_options->mutable_overlapping_submaps_trimmer_2d();
  options->set_fresh_submaps_count(
      options_dictionary->GetInt("fresh_submaps_count"));
  options->set_min_covered_area(
      options_dictionary->GetDouble("min_covered_area"));
  options->set_min_added_submaps_count(
      options_dictionary->GetInt("min_added_submaps_count"));
}

proto::PoseGraphOptions CreatePoseGraphOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::PoseGraphOptions options;
  options.set_optimize_every_n_nodes(
      parameter_dictionary->GetInt("optimize_every_n_nodes"));
  options.set_max_local_constraint_distance(
      parameter_dictionary->GetDouble("max_local_constraint_distance"));
  options.set_local_constraints_per_node(
      parameter_dictionary->GetDouble("local_constraints_per_node"));
  CHECK_GE(options.local_constraints_per_node(), 0.0);
  options.set_global_constraints_per_node(
      parameter_dictionary->GetDouble("global_constraints_per_node"));
  CHECK_GE(options.global_constraints_per_node(), 0.0);
  *options.mutable_constraint_builder_options() =
      constraints::CreateConstraintBuilderOptions(
          parameter_dictionary->GetDictionary("constraint_builder").get());
  options.set_matcher_translation_weight(
      parameter_dictionary->GetDouble("matcher_translation_weight"));
  options.set_matcher_rotation_weight(
      parameter_dictionary->GetDouble("matcher_rotation_weight"));
  *options.mutable_optimization_problem_options() =
      optimization::CreateOptimizationProblemOptions(
          parameter_dictionary->GetDictionary("optimization_problem").get());
  options.set_max_num_final_iterations(
      parameter_dictionary->GetNonNegativeInt("max_num_final_iterations"));
  CHECK_GT(options.max_num_final_iterations(), 0);
  options.set_log_work_queue_size(
      parameter_dictionary->GetBool("log_work_queue_size"));
  options.set_log_constraints(
      parameter_dictionary->GetBool("log_constraints"));
  options.set_log_residual_histograms(
      parameter_dictionary->GetBool("log_residual_histograms"));
  options.set_log_number_of_trimmed_loops(
      parameter_dictionary->GetBool("log_number_of_trimmed_loops"));
  options.set_global_constraint_search_after_n_seconds(
      parameter_dictionary->GetDouble(
          "global_constraint_search_after_n_seconds"));
  PopulateOverlappingSubmapsTrimmerOptions2D(&options, parameter_dictionary);
  return options;
}

proto::PoseGraph PoseGraph::ToProto(bool include_unfinished_submaps) const {
  proto::PoseGraph proto;

  std::map<int, proto::Trajectory* const> trajectory_protos;
  const auto trajectory = [&proto, &trajectory_protos](
                              const int trajectory_id) -> proto::Trajectory* {
    if (trajectory_protos.count(trajectory_id) == 0) {
      auto* const trajectory_proto = proto.add_trajectory();
      trajectory_proto->set_trajectory_id(trajectory_id);
      CHECK(trajectory_protos.emplace(trajectory_id, trajectory_proto).second);
    }
    return trajectory_protos.at(trajectory_id);
  };

  std::set<mapping::SubmapId> unfinished_submaps;
  for (const auto& submap_id_data : GetAllSubmapData()) {
    proto::Trajectory* trajectory_proto =
        trajectory(submap_id_data.id.trajectory_id);
    if (!include_unfinished_submaps &&
        !submap_id_data.data.submap->insertion_finished()) {
      // Collect IDs of all unfinished submaps and skip them.
      unfinished_submaps.insert(submap_id_data.id);
      continue;
    }
    CHECK(submap_id_data.data.submap != nullptr);
    auto* const submap_proto = trajectory_proto->add_submap();
    submap_proto->set_submap_index(submap_id_data.id.submap_index);
    *submap_proto->mutable_pose() =
        transform::ToProto(submap_id_data.data.pose);
  }

  auto constraints_copy = constraints();
  std::set<mapping::NodeId> orphaned_nodes;
  proto.mutable_constraint()->Reserve(constraints_copy.size());
  for (auto it = constraints_copy.begin(); it != constraints_copy.end();) {
    if (!include_unfinished_submaps &&
        unfinished_submaps.count(it->submap_id) > 0) {
      // Skip all those constraints that refer to unfinished submaps and
      // remember the corresponding trajectory nodes as potentially orphaned.
      orphaned_nodes.insert(it->node_id);
      it = constraints_copy.erase(it);
      continue;
    }
    *proto.add_constraint() = cartographer::mapping::ToProto(*it);
    ++it;
  }

  if (!include_unfinished_submaps) {
    // Iterate over all constraints and remove trajectory nodes from
    // 'orphaned_nodes' that are not actually orphaned.
    for (const auto& constraint : constraints_copy) {
      orphaned_nodes.erase(constraint.node_id);
    }
  }

  for (const auto& node_id_data : GetTrajectoryNodes()) {
    proto::Trajectory* trajectory_proto =
        trajectory(node_id_data.id.trajectory_id);
    CHECK(node_id_data.data.constant_data != nullptr);
    auto* const node_proto = trajectory_proto->add_node();
    node_proto->set_node_index(node_id_data.id.node_index);
    node_proto->set_timestamp(
        common::ToUniversal(node_id_data.data.constant_data->time));
    *node_proto->mutable_pose() =
        transform::ToProto(node_id_data.data.global_pose);
  }

  auto landmarks_copy = GetLandmarkPoses();
  proto.mutable_landmark_poses()->Reserve(landmarks_copy.size());
  for (const auto& id_pose : landmarks_copy) {
    auto* landmark_proto = proto.add_landmark_poses();
    landmark_proto->set_landmark_id(id_pose.first);
    *landmark_proto->mutable_global_pose() = transform::ToProto(id_pose.second);
  }

  std::map<std::string, std::set<int>> maps_data = GetMapsData();
  proto::PoseGraphMaps maps_proto;
  for (const auto& [map_name, trajectory_ids] : maps_data) {
    auto* map = maps_proto.add_map();
    map->set_name(map_name);
    for (int trajectory_id : trajectory_ids) {
      map->add_trajectory_id(trajectory_id);
    }
  }
  *proto.mutable_maps() = maps_proto;

  return proto;
}

}  // namespace mapping
}  // namespace cartographer
