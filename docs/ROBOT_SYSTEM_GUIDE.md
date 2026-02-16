# Robot System Guide

## Purpose

This document explains the robot runtime architecture in `vexHEHe` so code changes stay predictable during competition.

## Core File Map

- Competition entry: `src/main.cpp`
- Motion controllers: `src/auton/motion.cpp`
- Motion diagnostics: `src/auton/diagnostics.cpp`
- Drivetrain hardware layer: `src/subsystems/drivetrain.cpp`
- Sensor hardware layer: `src/subsystems/sensors.cpp`
- Localization task: `src/subsystems/localization.cpp`
- Hardware constants: `include/robot/ports.h`

## Control Flow

1. `initialize()`
- Initializes drivetrain, intake, indexer, sensors.
- Waits for IMU calibration completion.
- Resets and starts localization.

2. `competition_initialize()`
- Resets pose to a known origin before match behavior starts.

3. `autonomous()`
- Executes go-to-point segments with `auton::go_to_point_inches(...)`.
- Emits per-segment result and final pose summary.

4. `opcontrol()`
- Reads tank inputs (`left_y`, `right_y`) with deadband.
- Runs intake/indexer mode logic.
- Tracks and prints loop timing every 2 seconds.

## Localization Model

Localization uses:
- IMU heading (`sensors::heading_deg()`)
- Forward tracking wheel distance (`sensors::odom_inches()`)

Pose integration happens in a 10 ms task:
- Heading is fused with an offset from `reset_pose`.
- Turn-induced distance from wheel lateral offset is compensated.
- Position updates: `dx = ds * cos(heading)`, `dy = ds * sin(heading)`.

## Motion Controllers

### `drive_distance_inches`
- Trapezoidal profile + PID.
- Uses odom inches as position feedback.
- Includes settle criteria, timeout, overshoot tracking, direction mismatch detection.

### `turn_angle_deg`
- Trapezoidal profile + PID.
- Uses wrapped IMU heading feedback.
- Includes settle criteria, timeout, overshoot tracking, direction mismatch detection.

### `go_to_point_inches`
- Distance + heading state-feedback loop.
- Optional reverse-driving behavior when heading error is large.
- Includes settle criteria, timeout, direction mismatch detection.

## Built-In Telemetry

- `PORT_MAP,...`
  - Printed at startup to validate configuration.
- `MOTION_TRACE_*`
  - Detailed autonomous motion logs.
- `LOOP_STATS,...`
  - Reports actual loop compute frequency.

## Hardware Constants Policy

Update only `include/robot/ports.h` for:
- Smart port assignments
- Motor reversal
- Tracking wheel geometry
- Driver deadband

Avoid hardcoding duplicates in subsystem files.
