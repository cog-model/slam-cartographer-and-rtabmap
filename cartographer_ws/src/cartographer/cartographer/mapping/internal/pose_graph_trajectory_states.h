#pragma once

#include "cartographer/mapping/trajectory_state.h"
#include "cartographer/mapping/internal/trajectory_connectivity_state.h"

#include "glog/logging.h"

namespace cartographer {
namespace mapping {

class PoseGraphTrajectoryStates {

public:
  void AddTrajectory(int trajectory_id) {
    CHECK(!ContainsTrajectory(trajectory_id));
    trajectory_states_[trajectory_id];
    trajectory_connectivity_state_.Add(trajectory_id);
  }

  bool ContainsTrajectory(int trajectory_id) const {
    return trajectory_states_.count(trajectory_id);
  }
  bool CanModifyTrajectory(int trajectory_id) const {
    CHECK(ContainsTrajectory(trajectory_id));
    return IsTrajectoryActive(trajectory_id);
  }

  bool IsTrajectoryActive(int trajectory_id) const {
    CHECK(ContainsTrajectory(trajectory_id));
    return trajectory_states_.at(trajectory_id).state == TrajectoryState::State::ACTIVE;
  }
  bool IsTrajectoryFinished(int trajectory_id) const {
    CHECK(ContainsTrajectory(trajectory_id));
    return trajectory_states_.at(trajectory_id).state == TrajectoryState::State::FINISHED;
  }
  bool IsTrajectoryFrozen(int trajectory_id) const {
    CHECK(ContainsTrajectory(trajectory_id));
    return trajectory_states_.at(trajectory_id).state == TrajectoryState::State::FROZEN;
  }
  bool IsTrajectoryDeleted(int trajectory_id) const {
    CHECK(ContainsTrajectory(trajectory_id));
    return trajectory_states_.at(trajectory_id).state == TrajectoryState::State::DELETED;
  }

  bool IsTrajectoryTransitionNone(int trajectory_id) const {
    CHECK(ContainsTrajectory(trajectory_id));
    return trajectory_states_.at(trajectory_id).transition ==
        TrajectoryState::Transition::NONE;
  }
  bool IsTrajectoryScheduledForFinish(int trajectory_id) const {
    CHECK(ContainsTrajectory(trajectory_id));
    return trajectory_states_.at(trajectory_id).transition ==
        TrajectoryState::Transition::SCHEDULED_FOR_FINISH;
  }
  bool IsTrajectoryScheduledForFreezing(int trajectory_id) const {
    CHECK(ContainsTrajectory(trajectory_id));
    return trajectory_states_.at(trajectory_id).transition ==
        TrajectoryState::Transition::SCHEDULED_FOR_FREEZING;
  }
  bool IsTrajectoryScheduledForDeletion(int trajectory_id) const {
    CHECK(ContainsTrajectory(trajectory_id));
    return trajectory_states_.at(trajectory_id).transition ==
        TrajectoryState::Transition::SCHEDULED_FOR_DELETION;
  }
  bool IsTrajectoryReadyForDeletion(int trajectory_id) const {
    CHECK(ContainsTrajectory(trajectory_id));
    return trajectory_states_.at(trajectory_id).transition ==
        TrajectoryState::Transition::READY_FOR_DELETION;
  }

  void ScheduleTrajectoryForFinish(int trajectory_id) {
    CHECK(
        IsTrajectoryActive(trajectory_id) &&
        IsTrajectoryTransitionNone(trajectory_id));
    trajectory_states_.at(trajectory_id).transition =
        TrajectoryState::Transition::SCHEDULED_FOR_FINISH;
  }
  void ScheduleTrajectoryForFreezing(int trajectory_id) {
    CHECK(
        IsTrajectoryActive(trajectory_id) &&
        IsTrajectoryTransitionNone(trajectory_id));
    trajectory_states_.at(trajectory_id).transition =
        TrajectoryState::Transition::SCHEDULED_FOR_FREEZING;
  }
  void ScheduleTrajectoryForDeletion(int trajectory_id) {
    CHECK(
        !IsTrajectoryDeleted(trajectory_id) &&
        !IsTrajectoryScheduledForDeletion(trajectory_id) &&
        !IsTrajectoryReadyForDeletion(trajectory_id));
    trajectory_states_.at(trajectory_id).transition =
        TrajectoryState::Transition::SCHEDULED_FOR_DELETION;
  }
  void PrepareTrajectoryForDeletion(int trajectory_id) {
    CHECK(
        !IsTrajectoryDeleted(trajectory_id) &&
        IsTrajectoryScheduledForDeletion(trajectory_id));
    trajectory_states_.at(trajectory_id).transition =
        TrajectoryState::Transition::READY_FOR_DELETION;
  }

  void FinishTrajectory(int trajectory_id) {
    CHECK(
        IsTrajectoryScheduledForFinish(trajectory_id) ||
        IsTrajectoryScheduledForDeletion(trajectory_id));
    trajectory_states_.at(trajectory_id).state = TrajectoryState::State::FINISHED;
    if (IsTrajectoryScheduledForFinish(trajectory_id)) {
      trajectory_states_.at(trajectory_id).transition = TrajectoryState::Transition::NONE;
    }
  }
  void FreezeTrajectory(int trajectory_id) {
    CHECK(
        IsTrajectoryScheduledForFreezing(trajectory_id) ||
        IsTrajectoryScheduledForDeletion(trajectory_id));
    trajectory_states_.at(trajectory_id).state = TrajectoryState::State::FROZEN;
    if (IsTrajectoryScheduledForFreezing(trajectory_id)) {
      trajectory_states_.at(trajectory_id).transition = TrajectoryState::Transition::NONE;
    }
  }
  void DeleteTrajectory(int trajectory_id) {
    CHECK(IsTrajectoryReadyForDeletion(trajectory_id));
    trajectory_states_.at(trajectory_id).state = TrajectoryState::State::DELETED;
    trajectory_states_.at(trajectory_id).transition = TrajectoryState::Transition::NONE;
  }

  common::Time LastConnectionTime(int trajectory_a, int trajectory_b) const {
    return trajectory_connectivity_state_.LastConnectionTime(
        trajectory_a, trajectory_b);
  }
  bool TransitivelyConnected(int trajectory_a, int trajectory_b) const {
    return trajectory_connectivity_state_.TransitivelyConnected(
        trajectory_a, trajectory_b);
  }
  void Connect(int trajectory_a, int trajectory_b, common::Time time) {
    return trajectory_connectivity_state_.Connect(
        trajectory_a, trajectory_b, time);
  }
  std::vector<std::vector<int>> Components() const {
    return trajectory_connectivity_state_.Components();
  }

  const TrajectoryState& state(int trajectory_id) const {
    CHECK(ContainsTrajectory(trajectory_id));
    return trajectory_states_.at(trajectory_id);
  }
  const std::map<int, TrajectoryState>& trajectory_states() const {
    return trajectory_states_;
  }

  std::map<int, TrajectoryState>::const_iterator begin() const {
    return trajectory_states_.cbegin();
  }
  std::map<int, TrajectoryState>::const_iterator end() const {
    return trajectory_states_.cend();
  }
  std::map<int, TrajectoryState>::const_iterator cbegin() const {
    return trajectory_states_.cbegin();
  }
  std::map<int, TrajectoryState>::const_iterator cend() const {
    return trajectory_states_.cend();
  }

private:
  std::map<int, TrajectoryState> trajectory_states_;
  TrajectoryConnectivityState trajectory_connectivity_state_;
};

}
}
