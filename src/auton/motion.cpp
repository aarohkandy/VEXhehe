#include "auton/motion.h"

#include <algorithm>
#include <cmath>

#include "api.h"
#include "subsystems/drivetrain.h"
#include "subsystems/sensors.h"

namespace {
constexpr std::uint32_t kLoopDtMs = 10;
constexpr double kLoopDtSec = 0.01;

struct Profile {
  double distance = 0.0;
  double max_v = 0.0;
  double max_a = 0.0;
  double t_acc = 0.0;
  double t_cruise = 0.0;
  double total_t = 0.0;
  double d_acc = 0.0;
};

struct Pid {
  double kP;
  double kI;
  double kD;
  double integral = 0.0;
  double previous_error = 0.0;
};

auton::MotionSummary g_last_summary{};

double clamp(double value, double lo, double hi) { return std::max(lo, std::min(value, hi)); }

double wrap_deg(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

Profile make_profile(double distance, double max_speed, double max_accel) {
  Profile p{};
  p.distance = std::abs(distance);
  p.max_v = std::max(1e-3, std::abs(max_speed));
  p.max_a = std::max(1e-3, std::abs(max_accel));

  const double t_to_max = p.max_v / p.max_a;
  const double d_to_max = 0.5 * p.max_a * t_to_max * t_to_max;

  if (2.0 * d_to_max >= p.distance) {
    p.t_acc = std::sqrt(p.distance / p.max_a);
    p.d_acc = 0.5 * p.max_a * p.t_acc * p.t_acc;
    p.t_cruise = 0.0;
    p.total_t = 2.0 * p.t_acc;
    p.max_v = p.max_a * p.t_acc;
  } else {
    p.t_acc = t_to_max;
    p.d_acc = d_to_max;
    const double d_cruise = p.distance - 2.0 * p.d_acc;
    p.t_cruise = d_cruise / p.max_v;
    p.total_t = (2.0 * p.t_acc) + p.t_cruise;
  }
  return p;
}

void sample_profile(const Profile& p, double t, double* s, double* v) {
  if (t <= 0.0) {
    *s = 0.0;
    *v = 0.0;
    return;
  }

  if (t < p.t_acc) {
    *s = 0.5 * p.max_a * t * t;
    *v = p.max_a * t;
    return;
  }

  if (t < (p.t_acc + p.t_cruise)) {
    const double tc = t - p.t_acc;
    *s = p.d_acc + (p.max_v * tc);
    *v = p.max_v;
    return;
  }

  if (t < p.total_t) {
    const double td = t - p.t_acc - p.t_cruise;
    *s = p.d_acc + (p.max_v * p.t_cruise) + (p.max_v * td) - (0.5 * p.max_a * td * td);
    *v = p.max_v - (p.max_a * td);
    return;
  }

  *s = p.distance;
  *v = 0.0;
}

double pid_step(Pid* pid, double error, double dt) {
  pid->integral += error * dt;
  const double derivative = (error - pid->previous_error) / dt;
  pid->previous_error = error;
  return (pid->kP * error) + (pid->kI * pid->integral) + (pid->kD * derivative);
}

bool sign_mismatch(double cmd, double measured_rate) {
  return (std::abs(cmd) > 25.0) && (std::abs(measured_rate) > 1e-3) && ((cmd * measured_rate) < 0.0);
}

void set_result_by_priority(auton::MotionSummary* summary, bool timeout, bool overshoot) {
  if (summary->direction_mismatch) {
    summary->result = auton::MotionResult::kDirectionMismatch;
  } else if (overshoot) {
    summary->result = auton::MotionResult::kOvershoot;
  } else if (summary->settled) {
    summary->result = auton::MotionResult::kSettled;
  } else if (timeout) {
    summary->result = auton::MotionResult::kTimeout;
  } else {
    summary->result = auton::MotionResult::kTimeout;
  }
}
}  // namespace

namespace auton {
MotionSummary drive_distance_inches(double target_inches, const MotionConstraints& constraints) {
  drivetrain::tare_positions();
  sensors::tare_odom();
  diag::begin_motion_trace("drive_distance_inches", MotionKind::kDrive);

  const double sign = target_inches >= 0.0 ? 1.0 : -1.0;
  const Profile profile = make_profile(target_inches, constraints.max_speed, constraints.max_accel);
  Pid pid{3.0, 0.0, 0.8};

  std::uint32_t t_ms = 0;
  std::uint32_t settled_ms = 0;
  int mismatch_samples = 0;
  bool overshot = false;

  MotionSummary summary{};
  summary.max_overshoot = 0.0;

  while (t_ms < constraints.timeout_ms) {
    const double elapsed_s = static_cast<double>(t_ms) / 1000.0;
    double prof_s = 0.0;
    double prof_v = 0.0;
    sample_profile(profile, elapsed_s, &prof_s, &prof_v);

    const double target_pos = sign * prof_s;
    const double measured_pos = sensors::odom_inches();
    const double error = target_pos - measured_pos;

    const double ff = sign * prof_v * 2.2;
    const double pid_out = pid_step(&pid, error, kLoopDtSec);
    const double output = clamp(ff + pid_out, -127.0, 127.0);
    summary.max_command_abs = std::max(summary.max_command_abs, std::abs(output));

    drivetrain::set_tank(static_cast<int>(output), static_cast<int>(output));

    const double measured_rate = drivetrain::average_velocity_rpm();
    summary.peak_measured_rate = std::max(summary.peak_measured_rate, std::abs(measured_rate));
    if (sign_mismatch(output, measured_rate)) mismatch_samples++;
    if (mismatch_samples >= 15) summary.direction_mismatch = true;
    summary.mismatch_samples = mismatch_samples;

    const double terminal_error = (sign * target_inches) - measured_pos;
    if ((sign * measured_pos) > std::abs(target_inches)) {
      const double overshoot = std::abs((sign * measured_pos) - std::abs(target_inches));
      summary.max_overshoot = std::max(summary.max_overshoot, overshoot);
      if (overshoot > constraints.overshoot_error) overshot = true;
    }

    if (std::abs(terminal_error) <= constraints.settle_error) {
      settled_ms += kLoopDtMs;
      if (settled_ms >= constraints.settle_time_ms) {
        summary.settled = true;
        summary.final_error = terminal_error;
        summary.settle_entry_error = terminal_error;
        break;
      }
    } else {
      settled_ms = 0;
    }

    diag::log_motion_sample(MotionTraceSample{
        .kind = MotionKind::kDrive,
        .t_ms = t_ms,
        .target = target_pos,
        .measured = measured_pos,
        .error = error,
        .command = output,
        .measured_rate = measured_rate,
        .direction_mismatch = sign_mismatch(output, measured_rate),
    });

    t_ms += kLoopDtMs;
    pros::delay(kLoopDtMs);
  }

  drivetrain::stop();
  summary.elapsed_ms = t_ms;
  summary.final_error = (sign * target_inches) - sensors::odom_inches();
  set_result_by_priority(&summary, t_ms >= constraints.timeout_ms, overshot);
  g_last_summary = summary;
  diag::end_motion_trace(summary);
  return summary;
}

MotionSummary turn_angle_deg(double target_degrees, const MotionConstraints& constraints) {
  sensors::reset_heading();
  diag::begin_motion_trace("turn_angle_deg", MotionKind::kTurn);

  const double sign = target_degrees >= 0.0 ? 1.0 : -1.0;
  const Profile profile = make_profile(target_degrees, constraints.max_speed, constraints.max_accel);
  Pid pid{1.5, 0.0, 0.16};

  std::uint32_t t_ms = 0;
  std::uint32_t settled_ms = 0;
  int mismatch_samples = 0;
  bool overshot = false;

  MotionSummary summary{};
  summary.max_overshoot = 0.0;

  double last_heading = sensors::heading_deg();

  while (t_ms < constraints.timeout_ms) {
    const double elapsed_s = static_cast<double>(t_ms) / 1000.0;
    double prof_s = 0.0;
    double prof_v = 0.0;
    sample_profile(profile, elapsed_s, &prof_s, &prof_v);

    const double target_heading = sign * prof_s;
    const double measured_heading = wrap_deg(sensors::heading_deg());
    const double error = wrap_deg(target_heading - measured_heading);

    const double ff = sign * prof_v * 0.5;
    const double pid_out = pid_step(&pid, error, kLoopDtSec);
    const double output = clamp(ff + pid_out, -127.0, 127.0);
    summary.max_command_abs = std::max(summary.max_command_abs, std::abs(output));

    drivetrain::set_tank(static_cast<int>(-output), static_cast<int>(output));

    const double heading_rate = wrap_deg(measured_heading - last_heading) / kLoopDtSec;
    last_heading = measured_heading;
    summary.peak_measured_rate = std::max(summary.peak_measured_rate, std::abs(heading_rate));

    if (sign_mismatch(output, heading_rate)) mismatch_samples++;
    if (mismatch_samples >= 10) summary.direction_mismatch = true;
    summary.mismatch_samples = mismatch_samples;

    const double terminal_error = wrap_deg(target_degrees - measured_heading);
    if ((sign * measured_heading) > std::abs(target_degrees)) {
      const double overshoot = std::abs((sign * measured_heading) - std::abs(target_degrees));
      summary.max_overshoot = std::max(summary.max_overshoot, overshoot);
      if (overshoot > constraints.overshoot_error) overshot = true;
    }

    if (std::abs(terminal_error) <= constraints.settle_error) {
      settled_ms += kLoopDtMs;
      if (settled_ms >= constraints.settle_time_ms) {
        summary.settled = true;
        summary.final_error = terminal_error;
        summary.settle_entry_error = terminal_error;
        break;
      }
    } else {
      settled_ms = 0;
    }

    diag::log_motion_sample(MotionTraceSample{
        .kind = MotionKind::kTurn,
        .t_ms = t_ms,
        .target = target_heading,
        .measured = measured_heading,
        .error = error,
        .command = output,
        .measured_rate = heading_rate,
        .direction_mismatch = sign_mismatch(output, heading_rate),
    });

    t_ms += kLoopDtMs;
    pros::delay(kLoopDtMs);
  }

  drivetrain::stop();
  summary.elapsed_ms = t_ms;
  summary.final_error = wrap_deg(target_degrees - wrap_deg(sensors::heading_deg()));
  set_result_by_priority(&summary, t_ms >= constraints.timeout_ms, overshot);
  g_last_summary = summary;
  diag::end_motion_trace(summary);
  return summary;
}

void wait_ms(std::uint32_t duration_ms) { pros::delay(duration_ms); }

const MotionSummary& last_motion_summary(void) { return g_last_summary; }
}  // namespace auton
