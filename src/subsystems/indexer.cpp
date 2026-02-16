#include "subsystems/indexer.h"

#include "api.h"
#include "robot/ports.h"

namespace {
pros::Motor indexer_motor(robot::ports::kIndexerReversed ? -robot::ports::kIndexerMotor
                                                         : robot::ports::kIndexerMotor,
                          pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

int indexer_velocity(indexer::Mode mode) {
  switch (mode) {
    case indexer::Mode::kFeedForward:
      return 500;
    case indexer::Mode::kFeedReverse:
      return -400;
    case indexer::Mode::kOff:
    default:
      return 0;
  }
}

indexer::Mode current_mode = indexer::Mode::kOff;
}  // namespace

namespace indexer {
void init(void) { current_mode = Mode::kOff; }

void set_mode(Mode mode) { current_mode = mode; }

Mode mode(void) { return current_mode; }

void update(void) { indexer_motor.move_velocity(indexer_velocity(current_mode)); }
}  // namespace indexer
