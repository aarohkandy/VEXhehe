#include "auton/motion.h"

#include <algorithm>
#include <cmath>

#include "api.h"
#include "subsystems/drivetrain.h"
#include "subsystems/localization.h"
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

struct LoopStats {
  std::uint32_t iterations = 0;
  std::uint32_t min_us = 0xFFFFFFFFu;
  std::uint32_t max_us = 0;
  std::uint64_t total_us = 0;
};

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

int clamp_cmd(double cmd) { return static_cast<int>(clamp(cmd, -127.0, 127.0)); }
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
  LoopStats loop_stats{};

  MotionSummary summary{};
  summary.max_overshoot = 0.0;

  while (t_ms < constraints.timeout_ms) {
    const std::uint32_t start_us = pros::micros();
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

    drivetrain::set_tank(clamp_cmd(output), clamp_cmd(output));

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
    capture_loop_time(&loop_stats, pros::micros() - start_us);
    pros::delay(kLoopDtMs);
  }

  drivetrain::stop();
  summary.elapsed_ms = t_ms;
  summary.final_error = (sign * target_inches) - sensors::odom_inches();
  set_result_by_priority(&summary, t_ms >= constraints.timeout_ms, overshot);
  g_last_summary = summary;
  diag::end_motion_trace(summary);
  print_loop_stats("drive_distance", loop_stats);
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
  LoopStats loop_stats{};

  MotionSummary summary{};
  summary.max_overshoot = 0.0;

  double last_heading = sensors::heading_deg();

  while (t_ms < constraints.timeout_ms) {
    const std::uint32_t start_us = pros::micros();
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

    drivetrain::set_tank(clamp_cmd(-output), clamp_cmd(output));

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
    capture_loop_time(&loop_stats, pros::micros() - start_us);
    pros::delay(kLoopDtMs);
  }

  drivetrain::stop();
  summary.elapsed_ms = t_ms;
  summary.final_error = wrap_deg(target_degrees - wrap_deg(sensors::heading_deg()));
  set_result_by_priority(&summary, t_ms >= constraints.timeout_ms, overshot);
  g_last_summary = summary;
  diag::end_motion_trace(summary);
  print_loop_stats("turn_angle", loop_stats);
  return summary;
}

MotionSummary go_to_point_inches(double x_in, double y_in, const GoToPointConstraints& constraints) {
  diag::begin_motion_trace("go_to_point_inches", MotionKind::kGoToPoint);

  Pid linear_pid{constraints.linear_kp, 0.0, constraints.linear_kd};
  Pid heading_pid{constraints.heading_kp, 0.0, constraints.heading_kd};

  std::uint32_t t_ms = 0;
  std::uint32_t settled_ms = 0;
  int mismatch_samples = 0;
  LoopStats loop_stats{};

  MotionSummary summary{};
  summary.max_overshoot = 0.0;

  double previous_distance = 0.0;
  bool previous_distance_set = false;

  while (t_ms < constraints.timeout_ms) {
    const std::uint32_t start_us = pros::micros();
    const localization::Pose state = localization::pose();

    const double dx = x_in - state.x_in;
    const double dy = y_in - state.y_in;
    const double distance = std::hypot(dx, dy);
    const double target_heading = std::atan2(dy, dx) * (180.0 / 3.14159265358979323846);
    double heading_error = wrap_deg(target_heading - state.heading_deg);
    double drive_direction = 1.0;

    if (constraints.allow_reverse && std::abs(heading_error) > 95.0) {
      drive_direction = -1.0;
      heading_error = wrap_deg(heading_error > 0.0 ? heading_error - 180.0 : heading_error + 180.0);
    }

    const double forward_cmd =
        clamp(drive_direction * pid_step(&linear_pid, distance, kLoopDtSec), -constraints.max_forward_cmd,
              constraints.max_forward_cmd);
    const double turn_cmd =
        clamp(pid_step(&heading_pid, heading_error, kLoopDtSec), -constraints.max_turn_cmd,
              constraints.max_turn_cmd);

    const double left_cmd = forward_cmd - turn_cmd;
    const double right_cmd = forward_cmd + turn_cmd;
    drivetrain::set_tank(clamp_cmd(left_cmd), clamp_cmd(right_cmd));
    summary.max_command_abs = std::max(summary.max_command_abs, std::max(std::abs(left_cmd), std::abs(right_cmd)));

    const double measured_rate = drivetrain::average_velocity_rpm();
    summary.peak_measured_rate = std::max(summary.peak_measured_rate, std::abs(measured_rate));
    if (sign_mismatch(forward_cmd, measured_rate)) mismatch_samples++;
    if (mismatch_samples >= 15) summary.direction_mismatch = true;
    summary.mismatch_samples = mismatch_samples;

    if (previous_distance_set && (previous_distance - distance) < -constraints.settle_distance_in) {
      summary.max_overshoot = std::max(summary.max_overshoot, distance - previous_distance);
    }
    previous_distance = distance;
    previous_distance_set = true;

    if ((distance <= constraints.settle_distance_in) &&
        (std::abs(heading_error) <= constraints.settle_heading_deg)) {
      settled_ms += kLoopDtMs;
      if (settled_ms >= constraints.settle_time_ms) {
        summary.settled = true;
        summary.final_error = distance;
        summary.settle_entry_error = distance;
        break;
      }
    } else {
      settled_ms = 0;
    }

    diag::log_motion_sample(MotionTraceSample{
        .kind = MotionKind::kGoToPoint,
        .t_ms = t_ms,
        .target = 0.0,
        .measured = distance,
        .error = distance,
        .command = forward_cmd,
        .measured_rate = measured_rate,
        .direction_mismatch = sign_mismatch(forward_cmd, measured_rate),
    });

    t_ms += kLoopDtMs;
    capture_loop_time(&loop_stats, pros::micros() - start_us);
    pros::delay(kLoopDtMs);
  }

  drivetrain::stop();
  const localization::Pose final_state = localization::pose();
  summary.elapsed_ms = t_ms;
  summary.final_error = std::hypot(x_in - final_state.x_in, y_in - final_state.y_in);
  set_result_by_priority(&summary, t_ms >= constraints.timeout_ms, false);
  g_last_summary = summary;
  diag::end_motion_trace(summary);
  print_loop_stats("go_to_point", loop_stats);
  return summary;
}

void wait_ms(std::uint32_t duration_ms) { pros::delay(duration_ms); }

const MotionSummary& last_motion_summary(void) { return g_last_summary; }
}  // namespace auton
