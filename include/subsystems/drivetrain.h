#pragma once

namespace drivetrain {
void init(void);
void arcade(int forward, int turn);
void set_tank(int left, int right);
void stop(void);

void tare_positions(void);
double average_position_deg(void);
double average_velocity_rpm(void);
double left_velocity_rpm(void);
double right_velocity_rpm(void);
}  // namespace drivetrain
