#include "subsystems/drive.h"

#include "subsystems/drivetrain.h"

namespace drive {
void arcade(int forward, int turn) { drivetrain::arcade(forward, turn); }

void tank(int left, int right) { drivetrain::set_tank(left, right); }

void stop(void) { drivetrain::stop(); }
}  // namespace drive
