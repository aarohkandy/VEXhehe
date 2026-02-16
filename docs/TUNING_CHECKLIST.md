# On-Robot Tuning Checklist

Use this list after every major drivetrain, sensor, or weight change.

## 1. Wiring and Direction Sanity

1. Boot and read `PORT_MAP,...` logs.
2. Confirm each motor and sensor port matches the physical robot.
3. In driver control, verify:
   - both left motors spin forward on left stick forward
   - both right motors spin forward on right stick forward
4. If a side is wrong, fix reversal flags in `include/robot/ports.h`.

## 2. Sensor Sign Checks

1. Push robot forward by hand.
2. Confirm forward odom distance increases.
3. Rotate robot clockwise and counterclockwise.
4. Confirm heading direction is consistent with expected sign.

## 3. Localization Drift Check

1. Reset pose at start.
2. Drive a square slowly.
3. End near start point and inspect final pose print.
4. If in-place turns create fake translation, adjust `kOdomWheelLateralOffsetIn`.

## 4. Autonomous Controller Tuning

1. Start with conservative `GoToPointConstraints` in `src/main.cpp`.
2. Increase `linear_kp` until response is firm.
3. Add `linear_kd` to reduce overshoot.
4. Increase `heading_kp` until turns lock in.
5. Add `heading_kd` if heading oscillates.
6. Keep `max_forward_cmd` and `max_turn_cmd` limited to prevent tipping.

## 5. Loop Timing Stability

1. Watch `LOOP_STATS,...` in autonomous and opcontrol.
2. Confirm loop period is near target and does not spike heavily.
3. If spikes appear, remove heavy per-loop work and re-check.

## 6. Final Match Readiness

1. Run autonomous 5 times from identical start.
2. Verify result consistency and final pose repeatability.
3. Test with low battery condition.
4. Re-check critical telemetry:
   - no direction mismatch events
   - acceptable overshoot
   - stable loop stats
