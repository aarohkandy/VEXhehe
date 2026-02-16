# Simulation Guide

## Goal

The simulator in `sim/` is used to stress autonomous and drivetrain logic before robot testing.

## Setup

```bash
cd sim
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## Standard Runs

### Baseline scripted scenario
```bash
python run.py --scenario scenarios/default_auton.yaml --out output/default --duration 14 --no-realtime
```

### Waypoint course
```bash
python run.py --waypoint-path scenarios/complex_waypoint_course.yaml --out output/waypoints --duration 22 --no-realtime
```

### PROS-like autonomous module
```bash
python run.py --pros-module scenarios.pros_auton_example --out output/pros --duration 8 --no-realtime
```

### Full validation suite
```bash
python tools/validate_suite.py
```

## Outputs

Each run produces:
- `trace.csv`: time-series robot state and command trace
- `param_events.csv`: config hot-reload events
- `report.json`: summary metrics and diagnostics

Primary diagnostics in `report.json`:
- reverse motor mismatch events
- turn overshoot estimate
- stop drift estimate

## Key Tunables

- `sim/config/robot.yaml`
  - mass, inertia, wheel radius, track width, motor RPM, friction model
- `sim/config/noise.yaml`
  - IMU and odom noise, latency, dropout
- `sim/config/faults.yaml`
  - per-motor variability and deadband mismatch

## Current Hardware-Aligned Values

The simulator is aligned to current robot assumptions:
- Drive wheel diameter: 3.25 in
- Track width: 10.0 in
- Cartridge speed: 200 RPM (green)

If hardware changes, update `sim/config/robot.yaml` first.
