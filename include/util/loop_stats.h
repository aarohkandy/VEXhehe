#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>

namespace util {
struct LoopStats {
  std::uint32_t iterations = 0;
  std::uint32_t min_us = 0xFFFFFFFFu;
  std::uint32_t max_us = 0;
  std::uint64_t total_us = 0;
};

inline void capture_loop_time(LoopStats* stats, std::uint32_t dt_us) {
  stats->iterations++;
  stats->total_us += dt_us;
  stats->min_us = std::min(stats->min_us, dt_us);
  stats->max_us = std::max(stats->max_us, dt_us);
}

inline void print_loop_stats(const char* label, const LoopStats& stats) {
  if (stats.iterations == 0) return;
  const double avg_us = static_cast<double>(stats.total_us) / static_cast<double>(stats.iterations);
  const double hz = avg_us > 1e-6 ? 1e6 / avg_us : 0.0;
  std::printf("LOOP_STATS,label=%s,iters=%lu,avg_us=%.1f,min_us=%lu,max_us=%lu,hz=%.2f\n", label,
              static_cast<unsigned long>(stats.iterations), avg_us,
              static_cast<unsigned long>(stats.min_us), static_cast<unsigned long>(stats.max_us), hz);
}
}  // namespace util
