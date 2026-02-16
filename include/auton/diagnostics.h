#pragma once

#include <cstdint>

namespace auton {
enum class MotionResult {
  kSettled = 0,
  kTimeout,
  kOvershoot,
  kDirectionMismatch,
};

struct MotionSummary {
  MotionResult result = MotionResult::kTimeout;
  std::uint32_t elapsed_ms = 0;
  double max_overshoot = 0.0;
  double final_error = 0.0;
  bool settled = false;
  bool direction_mismatch = false;
  int mismatch_samples = 0;
  double max_command_abs = 0.0;
  double peak_measured_rate = 0.0;
  double settle_entry_error = 0.0;
};

enum class MotionKind {
  kDrive = 0,
  kTurn,
};

struct MotionTraceSample {
  MotionKind kind = MotionKind::kDrive;
  std::uint32_t t_ms = 0;
  double target = 0.0;
  double measured = 0.0;
  double error = 0.0;
  double command = 0.0;
  double measured_rate = 0.0;
  bool direction_mismatch = false;
};

namespace diag {
void begin_motion_trace(const char* label, MotionKind kind);
void log_motion_sample(const MotionTraceSample& sample);
void end_motion_trace(const MotionSummary& summary);
}  // namespace diag
}  // namespace auton
