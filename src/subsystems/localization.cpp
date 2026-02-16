#include "subsystems/localization.h"

#include <cmath>

#include "api.h"
#include "robot/ports.h"
#include "subsystems/sensors.h"

namespace {
constexpr std::uint32_t kLoopDtMs = 10;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

pros::Mutex pose_mutex;
localization::Pose g_pose{};
double g_heading_offset_deg = 0.0;
double g_last_odom_in = 0.0;
double g_last_heading_deg = 0.0;

pros::Task* g_task = nullptr;
bool g_task_running = false;

double wrap_deg(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

double fused_heading_deg(void) {
  const double raw_heading = wrap_deg(sensors::heading_deg());
  return wrap_deg(raw_heading + g_heading_offset_deg);
}

void tracking_task(void*) {
  while (g_task_running) {
    const double odom_in = sensors::odom_inches();
    const double raw_delta_in = odom_in - g_last_odom_in;
    g_last_odom_in = odom_in;

    const double heading_deg = fused_heading_deg();
    const double delta_heading_deg = wrap_deg(heading_deg - g_last_heading_deg);
    g_last_heading_deg = heading_deg;

    // Subtract wheel travel induced by turning if tracking wheel is laterally offset from center.
    const double turn_induced_in =
        robot::ports::kOdomWheelLateralOffsetIn * (delta_heading_deg * kDegToRad);
    const double delta_in = raw_delta_in - turn_induced_in;
    const double heading_rad = heading_deg * kDegToRad;
    const double dx = delta_in * std::cos(heading_rad);
    const double dy = delta_in * std::sin(heading_rad);

    pose_mutex.take();
    g_pose.x_in += dx;
    g_pose.y_in += dy;
    g_pose.heading_deg = heading_deg;
    g_pose.timestamp_ms = pros::millis();
    pose_mutex.give();

    pros::delay(kLoopDtMs);
  }
}
}  // namespace

namespace localization {
void init(void) {
  if (g_task != nullptr) return;
  g_task_running = true;
  g_task = new pros::Task(tracking_task, nullptr, "loc_task");
}

void shutdown(void) {
  if (g_task == nullptr) return;
  g_task_running = false;
  pros::delay(kLoopDtMs + 2);
  delete g_task;
  g_task = nullptr;
}

void reset_pose(double x_in, double y_in, double heading_deg) {
  sensors::tare_odom();
  g_last_odom_in = sensors::odom_inches();
  g_heading_offset_deg = wrap_deg(heading_deg - wrap_deg(sensors::heading_deg()));
  g_last_heading_deg = wrap_deg(heading_deg);

  pose_mutex.take();
  g_pose.x_in = x_in;
  g_pose.y_in = y_in;
  g_pose.heading_deg = wrap_deg(heading_deg);
  g_pose.timestamp_ms = pros::millis();
  pose_mutex.give();
}

Pose pose(void) {
  pose_mutex.take();
  const Pose snapshot = g_pose;
  pose_mutex.give();
  return snapshot;
}
}  // namespace localization
