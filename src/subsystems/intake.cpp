#include "subsystems/intake.h"

#include "api.h"
#include "robot/ports.h"

namespace {
pros::Motor intake_main(robot::ports::kIntakeMainReversed ? -robot::ports::kIntakeMainMotor
                                                          : robot::ports::kIntakeMainMotor,
                        pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor intake_helper(robot::ports::kIntakeHelperReversed ? -robot::ports::kIntakeHelperMotor
                                                              : robot::ports::kIntakeHelperMotor,
                          pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

int mode_velocity(intake::Mode mode) {
  switch (mode) {
    case intake::Mode::kCollect:
      return 600;
    case intake::Mode::kReverse:
      return -600;
    case intake::Mode::kOff:
    default:
      return 0;
  }
}

intake::Mode current_mode = intake::Mode::kOff;
}  // namespace

namespace intake {
void init(void) { current_mode = Mode::kOff; }

void set_mode(Mode mode) { current_mode = mode; }

Mode mode(void) { return current_mode; }

void update(void) {
  const int velocity = mode_velocity(current_mode);
  intake_main.move_velocity(velocity);
  intake_helper.move_velocity(velocity);
}
}  // namespace intake
