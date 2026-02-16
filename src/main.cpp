#include "main.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "auton/motion.h"
#include "robot/ports.h"
#include "subsystems/drivetrain.h"
#include "subsystems/indexer.h"
#include "subsystems/intake.h"
#include "subsystems/drive.h"
#include "subsystems/localization.h"
#include "subsystems/sensors.h"

namespace {
struct LoopStats {
  std::uint32_t iterations = 0;
  std::uint32_t min_us = 0xFFFFFFFFu;
  std::uint32_t max_us = 0;
  std::uint64_t total_us = 0;
};

int apply_deadband(int input, int deadband) { return (std::abs(input) < deadband) ? 0 : input; }

void log_port_map(void) {
  printf("PORT_MAP,LF=%d,LM=%d,LB=%d,RF=%d,RM=%d,RB=%d,IMU=%d,ODOM=%d\n",
         robot::ports::kLeftDriveFrontMotor, robot::ports::kLeftDriveMiddleMotor,
         robot::ports::kLeftDriveBackMotor, robot::ports::kRightDriveFrontMotor,
         robot::ports::kRightDriveMiddleMotor, robot::ports::kRightDriveBackMotor,
         robot::ports::kImuPort, robot::ports::kOdomRotationPort);
  printf("PORT_MAP_REV,left=%d,right=%d\n", robot::ports::kLeftDriveReversed ? 1 : 0,
         robot::ports::kRightDriveReversed ? 1 : 0);
}

void capture_loop_time(LoopStats* stats, std::uint32_t dt_us) {
  stats->iterations++;
  stats->total_us += dt_us;
  stats->min_us = std::min(stats->min_us, dt_us);
  stats->max_us = std::max(stats->max_us, dt_us);
}

void print_loop_stats(const char* label, const LoopStats& stats) {
  if (stats.iterations == 0) return;
  const double avg_us = static_cast<double>(stats.total_us) / static_cast<double>(stats.iterations);
  const double hz = avg_us > 1e-6 ? 1e6 / avg_us : 0.0;
  printf("LOOP_STATS,label=%s,iters=%lu,avg_us=%.1f,min_us=%lu,max_us=%lu,hz=%.2f\n", label,
         static_cast<unsigned long>(stats.iterations), avg_us, static_cast<unsigned long>(stats.min_us),
         static_cast<unsigned long>(stats.max_us), hz);
}
}  // namespace

void initialize(void) {
  log_port_map();
  drivetrain::init();
  intake::init();
  indexer::init();
  sensors::init();
  while (sensors::imu_calibrating()) pros::delay(20);
  localization::reset_pose(0.0, 0.0, 0.0);
  localization::init();
}

void disabled(void) { drive::stop(); }

void competition_initialize(void) {
  localization::reset_pose(0.0, 0.0, 0.0);
  printf("COMP_INIT,ready=1\n");
}

void autonomous(void) {
  localization::reset_pose(0.0, 0.0, 0.0);

  auton::GoToPointConstraints gtp{};
  gtp.timeout_ms = 3600;
  gtp.settle_time_ms = 250;
  gtp.settle_distance_in = 1.0;
  gtp.settle_heading_deg = 5.0;
  gtp.linear_kp = 7.8;
  gtp.linear_kd = 0.9;
  gtp.heading_kp = 2.2;
  gtp.heading_kd = 0.14;
  gtp.max_forward_cmd = 95.0;
  gtp.max_turn_cmd = 80.0;

  const auton::MotionSummary p1 = auton::go_to_point_inches(28.0, 0.0, gtp);
  const auton::MotionSummary p2 = auton::go_to_point_inches(48.0, 22.0, gtp);
  const auton::MotionSummary p3 = auton::go_to_point_inches(20.0, 36.0, gtp);

  const localization::Pose end_pose = localization::pose();
  printf("AUTO_SEG_1,result=%d,err=%.2f,t=%lu,mismatch=%d\n", static_cast<int>(p1.result),
         p1.final_error, static_cast<unsigned long>(p1.elapsed_ms), p1.mismatch_samples);
  printf("AUTO_SEG_2,result=%d,err=%.2f,t=%lu,mismatch=%d\n", static_cast<int>(p2.result),
         p2.final_error, static_cast<unsigned long>(p2.elapsed_ms), p2.mismatch_samples);
  printf("AUTO_SEG_3,result=%d,err=%.2f,t=%lu,mismatch=%d\n", static_cast<int>(p3.result),
         p3.final_error, static_cast<unsigned long>(p3.elapsed_ms), p3.mismatch_samples);
  printf("AUTO_POSE_FINAL,x=%.2f,y=%.2f,h=%.2f\n", end_pose.x_in, end_pose.y_in, end_pose.heading_deg);
}

void opcontrol(void) {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  constexpr std::uint32_t kLoopDtMs = 10;
  LoopStats driver_loop_stats{};
  std::uint32_t report_timer_ms = 0;

  while (true) {
    const std::uint32_t loop_start_us = pros::micros();
    const int left = apply_deadband(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 5);
    const int right = apply_deadband(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), 5);
    drive::tank(left, right);

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
    capture_loop_time(&driver_loop_stats, pros::micros() - loop_start_us);
    report_timer_ms += kLoopDtMs;
    if (report_timer_ms >= 2000) {
      print_loop_stats("opcontrol", driver_loop_stats);
      report_timer_ms = 0;
    }
    pros::delay(kLoopDtMs);
  }
}
