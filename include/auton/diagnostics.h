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
};
}  // namespace auton
