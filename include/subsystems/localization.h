#pragma once

#include <cstdint>

namespace localization {
struct Pose {
  double x_in = 0.0;
  double y_in = 0.0;
  double heading_deg = 0.0;
  std::uint32_t timestamp_ms = 0;
};

void init(void);
void shutdown(void);

void reset_pose(double x_in = 0.0, double y_in = 0.0, double heading_deg = 0.0);
Pose pose(void);
}  // namespace localization
