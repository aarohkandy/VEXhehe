#pragma once

namespace sensors {
void init(void);
bool imu_calibrating(void);
void reset_heading(void);
double heading_deg(void);

void tare_odom(void);
double odom_deg(void);
double odom_inches(void);

double optical_hue(void);
}  // namespace sensors
