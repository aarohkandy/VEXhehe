#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path


def main() -> None:
    p = argparse.ArgumentParser(description="Analyze simulator trace")
    p.add_argument("trace", type=str)
    args = p.parse_args()

    path = Path(args.trace)
    rows = []
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        print("Empty trace")
        return

    final = rows[-1]
    max_slip = max(max(float(r["slip_left_mps"]), float(r["slip_right_mps"])) for r in rows)
    max_speed = max((float(r["vx_mps"]) ** 2 + float(r["vy_mps"]) ** 2) ** 0.5 for r in rows)

    print(f"samples: {len(rows)}")
    print(f"final pose: x={float(final['x_m']):.3f} m, y={float(final['y_m']):.3f} m, theta={float(final['theta_rad']):.3f} rad")
    print(f"max speed: {max_speed:.3f} m/s")
    print(f"max slip: {max_slip:.3f} m/s")


if __name__ == "__main__":
    main()
