#pragma once

#include <vector>
#include <map>

#include "cartographer/mapping/id.h"
#include "cartographer/mapping/constraint.h"

namespace cartographer {
namespace mapping {

struct TrimmedLoop {
  SubmapId submap_id;
  NodeId node_id;
  float score;
};

class PoseGraphConstraints {
public:
  template <typename T>
  void InsertConstraint(T&& constraint) {
    if (constraint.tag == Constraint::INTER_SUBMAP) {
      UpdateLastLoopIndices(constraint);
    }
    constraints_.emplace_back(std::forward<T>(constraint));
  }

  template <typename T>
  void InsertConstraints(const T& begin, const T& end) {
    for (T it = begin; it != end; ++it) {
      const Constraint& loop = *it;
      if (loop.tag != Constraint::INTER_SUBMAP) {
        continue;
      }
      UpdateLastLoopIndices(loop);
    }
    constraints_.insert(constraints_.end(), begin, end);
  }

  template <typename T>
  void SetConstraints(T&& constraints) {
    last_loop_submap_index_for_trajectory_.clear();
    last_loop_node_index_for_trajectory_.clear();
    constraints_ = std::forward<T>(constraints);
    for (const TrimmedLoop& trimmed_loop : loops_from_trimmed_submaps_) {
      UpdateLastLoopIndices(trimmed_loop);
    }
    for (const Constraint& loop : constraints_) {
      if (loop.tag != Constraint::INTER_SUBMAP) {
        continue;
      }
      UpdateLastLoopIndices(loop);
    }
  }

  template <typename T>
  void InsertLoopFromTrimmedSubmap(T&& trimmed_loop) {
    loops_from_trimmed_submaps_.emplace_back(std::forward<T>(trimmed_loop));
  }

  void SetAccumRotationAndTravelledDistance(
      const NodeId& node_id, double accum_rotation, double travelled_distance);
  void SetFirstNodeIdForSubmap(
      const NodeId& first_node_id, const SubmapId& submap_id);

  void FixNode(const NodeId& node_id);

  const std::vector<Constraint>& constraints() const {
    return constraints_;
  }

  std::pair<double, double> GetAccumRotationAndTravelledDistanceWithLoops(
      const NodeId& node_1, const NodeId& node_2, float min_score) const;
  bool IsLoopLast(const Constraint& loop);

  std::vector<Constraint>::iterator begin() {
    return constraints_.begin();
  }
  std::vector<Constraint>::iterator end() {
    return constraints_.end();
  }
  std::vector<Constraint>::const_iterator begin() const {
    return constraints_.cbegin();
  }
  std::vector<Constraint>::const_iterator end() const {
    return constraints_.cend();
  }
  std::vector<Constraint>::const_iterator cbegin() const {
    return constraints_.cbegin();
  }
  std::vector<Constraint>::const_iterator cend() const {
    return constraints_.cend();
  }
  size_t size() const {
    return constraints_.size();
  }

private:
  template<typename T>
  void UpdateLastLoopIndices(const T& loop) {
    constexpr bool isConstraint = std::is_same<T, Constraint>::value;
    if constexpr (isConstraint) {
      CHECK(loop.tag == Constraint::INTER_SUBMAP);
    }

    auto submap_it =
        last_loop_submap_index_for_trajectory_.find(loop.submap_id.trajectory_id);
    if (submap_it != last_loop_submap_index_for_trajectory_.end()) {
      submap_it->second = std::max(submap_it->second, loop.submap_id.submap_index);
    } else {
      last_loop_submap_index_for_trajectory_.emplace(
          loop.submap_id.trajectory_id, loop.submap_id.submap_index);
    }

    auto node_it =
        last_loop_node_index_for_trajectory_.find(loop.node_id.trajectory_id);
    if (node_it != last_loop_node_index_for_trajectory_.end()) {
      node_it->second = std::max(node_it->second, loop.node_id.node_index);
    } else {
      last_loop_node_index_for_trajectory_.emplace(
          loop.node_id.trajectory_id, loop.node_id.node_index);
    }
  }

  std::pair<double, double> GetAccumRotationAndTravelledDistanceWithLoopsSameTrajectory(
      NodeId node_1, NodeId node_2, float min_score) const;
  std::pair<double, double> GetAccumRotationAndTravelledDistanceWithLoopsDifferentTrajectories(
      NodeId node_1, NodeId node_2, float min_score) const;

private:
  std::vector<Constraint> constraints_;
  std::vector<TrimmedLoop> loops_from_trimmed_submaps_;
  std::map<NodeId, std::pair<double, double>> accum_rotation_and_travelled_distance_;
  std::map<int, NodeId> trajectory_fixed_node_;
  std::map<SubmapId, NodeId> first_node_id_for_submap_;
  std::map<int, int> last_loop_submap_index_for_trajectory_;
  std::map<int, int> last_loop_node_index_for_trajectory_;
};

}
}
