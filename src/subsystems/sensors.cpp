#include "subsystems/sensors.h"

#include <cmath>

#include "api.h"
#include "robot/ports.h"

namespace {
pros::Imu imu(robot::ports::kImuPort);
pros::Rotation odom_rotation(robot::ports::kOdomRotationPort);
pros::Optical optical(robot::ports::kOpticalPort);

constexpr double kPi = 3.14159265358979323846;
}  // namespace

namespace sensors {
void init(void) {
  imu.reset();
  odom_rotation.reset();
}

bool imu_calibrating(void) { return imu.is_calibrating(); }

void reset_heading(void) { imu.tare_heading(); }

double heading_deg(void) { return imu.get_heading(); }

void tare_odom(void) { odom_rotation.reset(); }

double odom_deg(void) { return odom_rotation.get_position(); }

double odom_inches(void) {
  const double wheel_circumference = robot::ports::kOdomWheelDiameterIn * kPi;
  return (odom_deg() / 360.0) * wheel_circumference;
}

double optical_hue(void) { return optical.get_hue(); }
}  // namespace sensors
