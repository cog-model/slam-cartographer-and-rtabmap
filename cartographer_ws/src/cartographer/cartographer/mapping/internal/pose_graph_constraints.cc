#include "cartographer/mapping/internal/pose_graph_constraints.h"

#include <type_traits>

namespace cartographer {
namespace mapping {

static std::pair<double, double> operator+(
    const std::pair<double, double>& a, const std::pair<double, double>& b) {
  std::pair<double, double> sum(a.first + b.first, a.second + b.second);
  return sum;
}

void PoseGraphConstraints::SetAccumRotationAndTravelledDistance(
    const NodeId& node_id, double accum_rotation, double travelled_distance) {
  CHECK(accum_rotation_and_travelled_distance_.count(node_id) == 0);
  accum_rotation_and_travelled_distance_.emplace(
      node_id, std::make_pair(accum_rotation, travelled_distance));
}

void PoseGraphConstraints::SetFirstNodeIdForSubmap(
      const NodeId& first_node_id, const SubmapId& submap_id) {
  CHECK(first_node_id_for_submap_.count(submap_id) == 0);
  first_node_id_for_submap_.emplace(submap_id, first_node_id);
}

void PoseGraphConstraints::FixNode(const NodeId& node_id) {
  auto emplace_result =
      trajectory_fixed_node_.emplace(node_id.trajectory_id, node_id);
  const bool& emplaced = emplace_result.second;
  if (!emplaced) {
    auto& it = emplace_result.first;
    it->second = std::max(it->second, node_id);
  }
}

std::pair<double, double> PoseGraphConstraints::GetAccumRotationAndTravelledDistanceWithLoops(
    const NodeId& node_1, const NodeId& node_2, float min_score) const {
  if (node_1.trajectory_id == node_2.trajectory_id) {
    return GetAccumRotationAndTravelledDistanceWithLoopsSameTrajectory(
        node_1, node_2, min_score);
  } else {
    return GetAccumRotationAndTravelledDistanceWithLoopsDifferentTrajectories(
        node_1, node_2, min_score);
  }
}

std::pair<double, double> PoseGraphConstraints::GetAccumRotationAndTravelledDistanceWithLoopsSameTrajectory(
    NodeId node_1, NodeId node_2, float min_score) const {
  CHECK(node_1.trajectory_id == node_2.trajectory_id);
  if (node_1 == node_2) {
    return std::make_pair(0.0, 0.0);
  }
  if (node_2 < node_1) {
    std::swap(node_1, node_2);
  }
  CHECK(accum_rotation_and_travelled_distance_.count(node_1));
  CHECK(accum_rotation_and_travelled_distance_.count(node_2));
  auto [accum_rotation_at_node_1, travelled_distance_at_node_1] =
      accum_rotation_and_travelled_distance_.at(node_1);
  auto [accum_rotation_at_node_2, travelled_distance_at_node_2] =
      accum_rotation_and_travelled_distance_.at(node_2);
  double accum_rotation = accum_rotation_at_node_2 - accum_rotation_at_node_1;
  double travelled_distance = travelled_distance_at_node_2 - travelled_distance_at_node_1;
  CHECK(accum_rotation >= 0.0);
  CHECK(travelled_distance >= 0.0);
  for (const Constraint& loop : constraints_) {
    if (loop.tag == Constraint::INTRA_SUBMAP) {
      continue;
    }
    if (loop.score < min_score) {
      continue;
    }
    // check that loop lies on the same trajectory as node_1 and node_2
    if (loop.submap_id.trajectory_id != node_1.trajectory_id ||
        loop.node_id.trajectory_id != node_1.trajectory_id) {
      continue;
    }
    CHECK(first_node_id_for_submap_.count(loop.submap_id));
    NodeId loop_node_1 = first_node_id_for_submap_.at(loop.submap_id);
    NodeId loop_node_2 = loop.node_id;
    if (loop_node_2 < loop_node_1) {
      std::swap(loop_node_1, loop_node_2);
    }
    // check that loops intersect
    if (loop_node_2.node_index <= node_1.node_index ||
        loop_node_1.node_index >= node_2.node_index) {
      continue;
    }
    CHECK(accum_rotation_and_travelled_distance_.count(loop_node_1));
    CHECK(accum_rotation_and_travelled_distance_.count(loop_node_2));
    auto [accum_rotation_at_loop_node_1, travelled_distance_at_loop_node_1] =
        accum_rotation_and_travelled_distance_.at(loop_node_1);
    auto [accum_rotation_at_loop_node_2, travelled_distance_at_loop_node_2] =
        accum_rotation_and_travelled_distance_.at(loop_node_2);
    double accum_rotation_with_loop =
        std::abs(accum_rotation_at_loop_node_1 - accum_rotation_at_node_1) +
        std::abs(accum_rotation_at_loop_node_2 - accum_rotation_at_node_2);
    double travelled_distance_with_loop =
        std::abs(travelled_distance_at_loop_node_1 - travelled_distance_at_node_1) +
        std::abs(travelled_distance_at_loop_node_2 - travelled_distance_at_node_2);
    accum_rotation = std::min(accum_rotation, accum_rotation_with_loop);
    travelled_distance = std::min(travelled_distance, travelled_distance_with_loop);
  }
  return std::make_pair(accum_rotation, travelled_distance);
}

std::pair<double, double> PoseGraphConstraints::GetAccumRotationAndTravelledDistanceWithLoopsDifferentTrajectories(
    NodeId node_1, NodeId node_2, float min_score) const {
  CHECK(node_1.trajectory_id != node_2.trajectory_id);
  if (node_2 < node_1) {
    std::swap(node_1, node_2);
  }

  double accum_rotation = std::numeric_limits<double>::max();
  double travelled_distance = std::numeric_limits<double>::max();
  for (const Constraint& loop : constraints_) {
    if (loop.tag == Constraint::INTRA_SUBMAP) {
      continue;
    }
    if (loop.score < min_score) {
      continue;
    }
    // check that loop connects the same trajectories on which
    // node_1 and node_2 lie
    if (std::minmax(loop.submap_id.trajectory_id, loop.node_id.trajectory_id) !=
        std::minmax(node_1.trajectory_id, node_2.trajectory_id)) {
      continue;
    }
    CHECK(first_node_id_for_submap_.count(loop.submap_id));
    NodeId loop_node_1 = first_node_id_for_submap_.at(loop.submap_id);
    NodeId loop_node_2 = loop.node_id;
    if (loop_node_2 < loop_node_1) {
      std::swap(loop_node_1, loop_node_2);
    }
    auto [accum_rotation_with_loop, travelled_distance_with_loop] =
        GetAccumRotationAndTravelledDistanceWithLoopsSameTrajectory(node_1, loop_node_1, min_score) +
        GetAccumRotationAndTravelledDistanceWithLoopsSameTrajectory(node_2, loop_node_2, min_score);
    accum_rotation = std::min(accum_rotation, accum_rotation_with_loop);
    travelled_distance = std::min(travelled_distance, travelled_distance_with_loop);
  }

  for (const TrimmedLoop& loop : loops_from_trimmed_submaps_) {
    if (loop.score < min_score) {
      continue;
    }
    // check that loop connects the same trajectories on which
    // node_1 and node_2 lie
    if (std::minmax(loop.submap_id.trajectory_id, loop.node_id.trajectory_id) !=
        std::minmax(node_1.trajectory_id, node_2.trajectory_id)) {
      continue;
    }
    CHECK(first_node_id_for_submap_.count(loop.submap_id));
    NodeId loop_node_1 = first_node_id_for_submap_.at(loop.submap_id);
    NodeId loop_node_2 = loop.node_id;
    if (loop_node_2 < loop_node_1) {
      std::swap(loop_node_1, loop_node_2);
    }
    auto [accum_rotation_with_loop, travelled_distance_with_loop] =
        GetAccumRotationAndTravelledDistanceWithLoopsSameTrajectory(node_1, loop_node_1, min_score) +
        GetAccumRotationAndTravelledDistanceWithLoopsSameTrajectory(node_2, loop_node_2, min_score);
    accum_rotation = std::min(accum_rotation, accum_rotation_with_loop);
    travelled_distance = std::min(travelled_distance, travelled_distance_with_loop);
  }

  auto [accum_rotation_at_node_1, travelled_distance_at_node_1] =
      accum_rotation_and_travelled_distance_.at(node_1);
  auto [accum_rotation_at_node_2, travelled_distance_at_node_2] =
      accum_rotation_and_travelled_distance_.at(node_2);

  auto fixed_node_1_it = trajectory_fixed_node_.find(node_1.trajectory_id);
  if (fixed_node_1_it != trajectory_fixed_node_.end()) {
    const NodeId& fixed_node_1 = fixed_node_1_it->second;
    auto [accum_rotation_at_fixed_node_1, travelled_distance_at_fixed_node_1] =
        accum_rotation_and_travelled_distance_.at(fixed_node_1);
    accum_rotation = std::min(accum_rotation,
        std::abs(accum_rotation_at_fixed_node_1 - accum_rotation_at_node_1));
    travelled_distance = std::min(travelled_distance,
        std::abs(travelled_distance_at_fixed_node_1 - travelled_distance_at_node_1));
  }
  auto fixed_node_2_it = trajectory_fixed_node_.find(node_2.trajectory_id);
  if (fixed_node_2_it != trajectory_fixed_node_.end()) {\
    const NodeId& fixed_node_2 = fixed_node_2_it->second;
    auto [accum_rotation_at_fixed_node_2, travelled_distance_at_fixed_node_2] =
        accum_rotation_and_travelled_distance_.at(fixed_node_2);
    accum_rotation = std::min(accum_rotation,
        std::abs(accum_rotation_at_fixed_node_2 - accum_rotation_at_node_2));
    travelled_distance = std::min(travelled_distance,
        std::abs(travelled_distance_at_fixed_node_2 - travelled_distance_at_node_2));
  }
  return std::make_pair(accum_rotation, travelled_distance);
}

bool PoseGraphConstraints::IsLoopLast(const Constraint& loop) {
  CHECK(loop.tag == Constraint::INTER_SUBMAP);
  CHECK(last_loop_submap_index_for_trajectory_.count(loop.submap_id.trajectory_id));
  CHECK(last_loop_node_index_for_trajectory_.count(loop.node_id.trajectory_id));
  CHECK(last_loop_submap_index_for_trajectory_.at(loop.submap_id.trajectory_id) >=
      loop.submap_id.submap_index);
  CHECK(last_loop_node_index_for_trajectory_.at(loop.node_id.trajectory_id) >=
      loop.node_id.node_index);
  return
      (last_loop_submap_index_for_trajectory_.at(loop.submap_id.trajectory_id) ==
          loop.submap_id.submap_index) ||
      (last_loop_node_index_for_trajectory_.at(loop.node_id.trajectory_id) ==
          loop.node_id.node_index);
}

}
}
