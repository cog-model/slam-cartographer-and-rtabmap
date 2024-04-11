#pragma once

namespace cartographer {
namespace mapping {

struct TrajectoryState {
  enum struct State {
    ACTIVE,
    FINISHED,
    FROZEN,
    DELETED
  };
  enum struct Transition {
    NONE,
    SCHEDULED_FOR_FINISH,
    SCHEDULED_FOR_FREEZING,
    SCHEDULED_FOR_DELETION,
    READY_FOR_DELETION
  };

  State state = State::ACTIVE;
  Transition transition = Transition::NONE;
};

}
}
