# VEXhehe

Competition-ready PROS codebase plus a configurable VEX-style simulator.

This repo is organized so robot code and simulation can evolve together:
- Robot code is in `include/` and `src/`
- Simulator is in `sim/`
- Hardware constants are centralized in `include/robot/ports.h`

## Robot Quick Start

### 1. Build
```bash
make quick
```

### 2. Upload from PROS terminal
```bash
pros upload
```

### 3. Verify startup logs
On boot, this code prints:
- `PORT_MAP,...` for motor/sensor ports
- `PORT_MAP_REV,...` for left/right drivetrain reversal
- `COMP_INIT,ready=1` after competition init

If these values do not match hardware, stop and fix `include/robot/ports.h`.

## Competition Behavior

- `initialize()`
  - initializes all subsystems
  - waits for IMU calibration
  - starts localization task
- `autonomous()`
  - runs a three-leg go-to-point path
  - prints segment result/error/mismatch and final pose
- `opcontrol()`
  - tank drive (`left_y`, `right_y`)
  - intake/indexer controls on `R1`/`R2`
  - loop timing output every 2s via `LOOP_STATS,...`

## Motion and Diagnostic Logs

Autonomous controllers emit trace lines:
- `MOTION_TRACE_BEGIN,...`
- `MOTION_TRACE,...`
- `MOTION_TRACE_END,...`

Loop timing lines:
- `LOOP_STATS,label=...`

These logs are intended to catch:
- reversed motors
- overshoot and settling failures
- loop-rate regressions

## Simulator Quick Start

```bash
cd sim
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python run.py --scenario scenarios/default_auton.yaml --out output/demo --duration 10 --no-realtime
```

Useful runs:
```bash
python tools/validate_suite.py
python run.py --waypoint-path scenarios/complex_waypoint_course.yaml --out output/waypoints --duration 22 --no-realtime
python run.py --pros-module scenarios.pros_auton_example --out output/pros --duration 8 --no-realtime
```

## Documentation

- Robot system guide: `docs/ROBOT_SYSTEM_GUIDE.md`
- Simulator guide: `docs/SIMULATION_GUIDE.md`
- On-robot tuning checklist: `docs/TUNING_CHECKLIST.md`
