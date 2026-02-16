#include "main.h"

#include "auton/motion.h"
#include "subsystems/drivetrain.h"
#include "subsystems/indexer.h"
#include "subsystems/intake.h"
#include "subsystems/sensors.h"
#include "subsystems/drive.h"

void initialize(void) {
  drivetrain::init();
  intake::init();
  indexer::init();
  sensors::init();
  while (sensors::imu_calibrating()) pros::delay(20);
}

void disabled(void) {}

void competition_initialize(void) {}

void autonomous(void) {
  auton::MotionConstraints linear;
  linear.max_speed = 26.0;
  linear.max_accel = 60.0;
  linear.timeout_ms = 4000;
  linear.settle_error = 1.0;
  linear.settle_time_ms = 250;
  linear.overshoot_error = 3.0;

  auton::MotionConstraints turn;
  turn.max_speed = 120.0;
  turn.max_accel = 240.0;
  turn.timeout_ms = 2500;
  turn.settle_error = 1.0;
  turn.settle_time_ms = 200;
  turn.overshoot_error = 4.0;

  const auton::MotionSummary forward = auton::drive_distance_inches(24.0, linear);
  const auton::MotionSummary rotate = auton::turn_angle_deg(90.0, turn);
  const auton::MotionSummary reverse = auton::drive_distance_inches(-24.0, linear);

  printf("A1 result=%d err=%.2f over=%.2f t=%lu\n", static_cast<int>(forward.result),
         forward.final_error, forward.max_overshoot, static_cast<unsigned long>(forward.elapsed_ms));
  printf("A2 result=%d err=%.2f over=%.2f t=%lu\n", static_cast<int>(rotate.result),
         rotate.final_error, rotate.max_overshoot, static_cast<unsigned long>(rotate.elapsed_ms));
  printf("A3 result=%d err=%.2f over=%.2f t=%lu\n", static_cast<int>(reverse.result),
         reverse.final_error, reverse.max_overshoot, static_cast<unsigned long>(reverse.elapsed_ms));
}

void opcontrol(void) {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
    const int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    drive::arcade(forward, turn);

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake::set_mode(intake::Mode::kCollect);
      indexer::set_mode(indexer::Mode::kFeedForward);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake::set_mode(intake::Mode::kReverse);
      indexer::set_mode(indexer::Mode::kFeedReverse);
    } else {
      intake::set_mode(intake::Mode::kOff);
      indexer::set_mode(indexer::Mode::kOff);
    }

    intake::update();
    indexer::update();
    pros::delay(10);
  }
}
