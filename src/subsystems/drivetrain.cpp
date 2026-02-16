#include "subsystems/drivetrain.h"

#include <algorithm>

#include "api.h"
#include "robot/ports.h"

namespace {
pros::Motor left_front(robot::ports::kLeftDriveReversed ? -robot::ports::kLeftDriveFrontMotor
                                                         : robot::ports::kLeftDriveFrontMotor,
                       pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor left_middle(robot::ports::kLeftDriveReversed ? -robot::ports::kLeftDriveMiddleMotor
                                                          : robot::ports::kLeftDriveMiddleMotor,
                        pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor left_back(robot::ports::kLeftDriveReversed ? -robot::ports::kLeftDriveBackMotor
                                                        : robot::ports::kLeftDriveBackMotor,
                      pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

pros::Motor right_front(robot::ports::kRightDriveReversed ? -robot::ports::kRightDriveFrontMotor
                                                           : robot::ports::kRightDriveFrontMotor,
                        pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor right_middle(robot::ports::kRightDriveReversed ? -robot::ports::kRightDriveMiddleMotor
                                                            : robot::ports::kRightDriveMiddleMotor,
                         pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor right_back(robot::ports::kRightDriveReversed ? -robot::ports::kRightDriveBackMotor
                                                          : robot::ports::kRightDriveBackMotor,
                       pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

int clamp_power(int value) { return std::clamp(value, -127, 127); }
}  // namespace

namespace drivetrain {
void init(void) {
  left_front.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  left_middle.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  left_back.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  right_front.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  right_middle.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  right_back.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void set_tank(int left, int right) {
  const int left_cmd = clamp_power(left);
  const int right_cmd = clamp_power(right);

  left_front.move(left_cmd);
  left_middle.move(left_cmd);
  left_back.move(left_cmd);
  right_front.move(right_cmd);
  right_middle.move(right_cmd);
  right_back.move(right_cmd);
}

void arcade(int forward, int turn) { set_tank(forward + turn, forward - turn); }

void stop(void) { set_tank(0, 0); }

void tare_positions(void) {
  left_front.tare_position();
  left_middle.tare_position();
  left_back.tare_position();
  right_front.tare_position();
  right_middle.tare_position();
  right_back.tare_position();
}

double average_position_deg(void) {
  const double left_avg =
      (left_front.get_position() + left_middle.get_position() + left_back.get_position()) / 3.0;
  const double right_avg =
      (right_front.get_position() + right_middle.get_position() + right_back.get_position()) / 3.0;
  return (left_avg + right_avg) / 2.0;
}

double left_velocity_rpm(void) {
  return (left_front.get_actual_velocity() + left_middle.get_actual_velocity() +
          left_back.get_actual_velocity()) /
         3.0;
}

double right_velocity_rpm(void) {
  return (right_front.get_actual_velocity() + right_middle.get_actual_velocity() +
          right_back.get_actual_velocity()) /
         3.0;
}

double average_velocity_rpm(void) { return (left_velocity_rpm() + right_velocity_rpm()) / 2.0; }
}  // namespace drivetrain
