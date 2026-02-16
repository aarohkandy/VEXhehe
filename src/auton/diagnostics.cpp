#include "auton/diagnostics.h"

#include <cstdio>

namespace {
const char* motion_kind_name(auton::MotionKind kind) {
  switch (kind) {
    case auton::MotionKind::kDrive:
      return "drive";
    case auton::MotionKind::kTurn:
      return "turn";
    case auton::MotionKind::kGoToPoint:
      return "goto_point";
    default:
      return "unknown";
  }
}

const char* motion_result_name(auton::MotionResult result) {
  switch (result) {
    case auton::MotionResult::kSettled:
      return "settled";
    case auton::MotionResult::kTimeout:
      return "timeout";
    case auton::MotionResult::kOvershoot:
      return "overshoot";
    case auton::MotionResult::kDirectionMismatch:
      return "direction_mismatch";
    default:
      return "unknown";
  }
}

bool g_trace_active = false;
int g_sample_count = 0;
auton::MotionKind g_kind = auton::MotionKind::kDrive;
}  // namespace

namespace auton::diag {
void begin_motion_trace(const char* label, MotionKind kind) {
  g_trace_active = true;
  g_sample_count = 0;
  g_kind = kind;
  printf("MOTION_TRACE_BEGIN,label=%s,kind=%s\n", label ? label : "unnamed",
         motion_kind_name(kind));
}

void log_motion_sample(const MotionTraceSample& sample) {
  if (!g_trace_active) return;

  // Print every 5th sample (~50ms at 10ms loop) to keep logs readable.
  g_sample_count++;
  if ((g_sample_count % 5) != 0) return;

  printf(
      "MOTION_TRACE,kind=%s,t_ms=%lu,target=%.4f,measured=%.4f,error=%.4f,cmd=%.3f,rate=%.4f,mismatch=%d\n",
      motion_kind_name(sample.kind), static_cast<unsigned long>(sample.t_ms), sample.target,
      sample.measured, sample.error, sample.command, sample.measured_rate,
      sample.direction_mismatch ? 1 : 0);
}

void end_motion_trace(const MotionSummary& summary) {
  if (!g_trace_active) return;

  printf(
      "MOTION_TRACE_END,kind=%s,result=%s,elapsed_ms=%lu,final_error=%.4f,max_overshoot=%.4f,"
      "mismatch_samples=%d,max_cmd=%.3f,peak_rate=%.4f,settled=%d\n",
      motion_kind_name(g_kind), motion_result_name(summary.result),
      static_cast<unsigned long>(summary.elapsed_ms), summary.final_error, summary.max_overshoot,
      summary.mismatch_samples, summary.max_command_abs, summary.peak_measured_rate,
      summary.settled ? 1 : 0);

  g_trace_active = false;
}
}  // namespace auton::diag
