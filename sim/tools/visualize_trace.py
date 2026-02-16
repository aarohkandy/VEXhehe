#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Replay a simulator trace with matplotlib")
    p.add_argument("trace", type=str, help="Path to trace.csv")
    p.add_argument("--step", type=int, default=5, help="Draw every Nth sample")
    p.add_argument("--robot-length", type=float, default=0.35)
    p.add_argument("--robot-width", type=float, default=0.30)
    return p.parse_args()


def robot_outline(x: float, y: float, theta: float, length: float, width: float) -> tuple[list[float], list[float]]:
    hl = 0.5 * length
    hw = 0.5 * width
    local = [(+hl, +hw), (+hl, -hw), (-hl, -hw), (-hl, +hw)]
    c = math.cos(theta)
    s = math.sin(theta)
    pts = []
    for lx, ly in local:
        pts.append((x + lx * c - ly * s, y + lx * s + ly * c))
    xs = [p[0] for p in pts] + [pts[0][0]]
    ys = [p[1] for p in pts] + [pts[0][1]]
    return xs, ys


def main() -> None:
    args = parse_args()

    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required for visualization: {exc}")

    path = Path(args.trace)
    rows = []
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        raise SystemExit("Trace is empty")

    xs = [float(r["x_m"]) for r in rows]
    ys = [float(r["y_m"]) for r in rows]
    th = [float(r["theta_rad"]) for r in rows]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("VEX Trace Replay")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")

    ax.plot(xs, ys, linewidth=1.3, label="path")

    step = max(1, args.step)
    for i in range(0, len(xs), step):
        bx, by = robot_outline(xs[i], ys[i], th[i], args.robot_length, args.robot_width)
        ax.plot(bx, by, linewidth=0.8, alpha=0.35)

    bx, by = robot_outline(xs[-1], ys[-1], th[-1], args.robot_length, args.robot_width)
    ax.plot(bx, by, linewidth=2.0, label="final pose")

    heading_len = args.robot_length * 0.55
    hx = xs[-1] + heading_len * math.cos(th[-1])
    hy = ys[-1] + heading_len * math.sin(th[-1])
    ax.plot([xs[-1], hx], [ys[-1], hy], linewidth=2.0, label="heading")

    ax.legend(loc="upper right")
    plt.show()


if __name__ == "__main__":
    main()
