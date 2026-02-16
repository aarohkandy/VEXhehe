#pragma once

#include <cstdint>

#include "auton/diagnostics.h"

namespace auton {
struct MotionConstraints {
  double max_speed = 24.0;       // in/s for linear, deg/s for angular
  double max_accel = 60.0;       // in/s^2 for linear, deg/s^2 for angular
  std::uint32_t timeout_ms = 3000;
  double settle_error = 1.0;
  std::uint32_t settle_time_ms = 200;
  double overshoot_error = 4.0;
};

MotionSummary drive_distance_inches(double target_inches,
                                    const MotionConstraints& constraints = MotionConstraints{});
MotionSummary turn_angle_deg(double target_degrees,
                             const MotionConstraints& constraints = MotionConstraints{});
void wait_ms(std::uint32_t duration_ms);
const MotionSummary& last_motion_summary(void);
}  // namespace auton
