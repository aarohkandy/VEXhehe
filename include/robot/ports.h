#pragma once

namespace robot::ports {
// Drivetrain motors
constexpr int kLeftDriveFrontMotor = 13;
constexpr int kLeftDriveMiddleMotor = 14;
constexpr int kLeftDriveBackMotor = 15;
constexpr int kRightDriveFrontMotor = 17;
constexpr int kRightDriveMiddleMotor = 19;
constexpr int kRightDriveBackMotor = 20;

constexpr bool kLeftDriveReversed = true;
constexpr bool kRightDriveReversed = false;

// Mechanisms
constexpr int kIndexerMotor = 2;
constexpr int kIntakeHelperMotor = 3;
constexpr int kIntakeMainMotor = 21;

constexpr bool kIndexerReversed = true;
constexpr bool kIntakeHelperReversed = true;
constexpr bool kIntakeMainReversed = false;

// Sensors
constexpr int kImuPort = 6;
constexpr int kOdomRotationPort = 16;
constexpr int kOpticalPort = 4;

// Pneumatics (ADI)
constexpr char kGoalFlapAdiPort = 'A';
constexpr char kLoaderAdiPort = 'C';

// Odom conversion (forward tracking wheel)
// pushback_sbot uses LemLib Omniwheel::NEW_2 (2.125 in).
constexpr double kOdomWheelDiameterIn = 2.125;
// Lateral offset of forward tracking wheel from robot center (inches).
constexpr double kOdomWheelLateralOffsetIn = 0.085;

// Driver control shaping
constexpr int kTankJoystickDeadband = 10;
}  // namespace robot::ports
