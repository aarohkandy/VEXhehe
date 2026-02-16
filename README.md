# VEXhehe

Headless high-fidelity VEX simulator is in `sim/`.

Quick run:

```bash
cd sim
python3 -m pip install -r requirements.txt
python3 run.py --out output/demo --duration 10
```

PROS-style project scaffold for VEX V5.

## Structure

- `include/`: headers and robot config
- `src/`: competition lifecycle and subsystem implementation
- `src/subsystems/`: subsystem source files

## Next setup step

Install PROS CLI and initialize/build tooling in this folder:

```bash
pros c apply
pros make
```
