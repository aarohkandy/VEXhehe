# Headless VEX V5 Simulator

High-fidelity 2D headless simulator for a 6-motor VEX V5 omni tank drivetrain.

## Features
- Inertia-aware rigid-body dynamics (mass + moment of inertia)
- Orthotropic omni friction (longitudinal + lateral slip behavior)
- Per-motor mismatch and battery sag model
- IMU and rotation sensor noise/latency/dropout modeling
- Hot-reload for `sim/config/*.yaml` while simulation is running
- CSV traces + JSON summary report
- Built-in diagnostics for reversed motors and overshoot/drift
- Optional live visualization during simulation
- Minimal PROS-like Python shim for autonomous structure testing

## Quick Start
```bash
cd sim
python3 -m pip install -r requirements.txt
python3 run.py --out output/demo --duration 10
# live visualization
python3 run.py --out output/demo --duration 10 --visualize
```

Edit these during runtime to hot-reload dynamics/sensor behavior:
- `sim/config/robot.yaml`
- `sim/config/noise.yaml`
- `sim/config/faults.yaml`

## CLI
```bash
python3 run.py --out output/demo --duration 15
python3 run.py --scenario scenarios/default_auton.yaml --out output/run2
python3 run.py --pros-module scenarios.pros_auton_example --out output/pros_test
python3 run.py --scenario scenarios/default_auton.yaml --out output/vis --visualize
python3 tools/analyze_trace.py output/vis/trace.csv --report output/vis/report.json
python3 tools/visualize_trace.py output/vis/trace.csv
python3 tools/validate_suite.py
```

## Notes
- Realism is prioritized over speed.
- `--realtime` is enabled by default; use `--no-realtime` for fastest offline replay.
- `report.json` now includes a `diagnostics` section with:
  - reverse motor mismatch events
  - turn overshoot estimate (scripted turn segments)
  - stop drift estimate (scripted straight-then-stop segments)
- `tools/validate_suite.py` runs a multi-case regression:
  - clean baseline expected to pass
  - forced overshoot expected to trigger `turn_overshoot_high`
  - forced motor-sign fault expected to trigger `motor_direction_mismatch_detected`
