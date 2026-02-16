#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import pygame


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Replay sim trace in a live pygame window")
    p.add_argument("trace", type=str)
    p.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    p.add_argument("--width", type=int, default=980)
    p.add_argument("--height", type=int, default=720)
    return p.parse_args()


def load_trace(path: Path):
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append(
                (
                    float(r["time_s"]),
                    float(r["x_m"]),
                    float(r["y_m"]),
                    float(r["theta_rad"]),
                )
            )
    return rows


def main() -> None:
    args = parse_args()
    trace = load_trace(Path(args.trace))
    if not trace:
        raise SystemExit("Empty trace")

    min_x = min(r[1] for r in trace)
    max_x = max(r[1] for r in trace)
    min_y = min(r[2] for r in trace)
    max_y = max(r[2] for r in trace)

    pad = 0.4
    span_x = max(1.0, (max_x - min_x) + 2 * pad)
    span_y = max(1.0, (max_y - min_y) + 2 * pad)

    pygame.init()
    screen = pygame.display.set_mode((args.width, args.height))
    pygame.display.set_caption("VEX Simulation Replay")
    font = pygame.font.SysFont("monospace", 18)
    clock = pygame.time.Clock()

    bg = (18, 22, 28)
    fg = (220, 227, 235)
    path_c = (70, 160, 255)
    bot_c = (255, 185, 70)
    head_c = (255, 85, 85)

    def to_px(x: float, y: float):
        nx = ((x - (min_x - pad)) / span_x)
        ny = ((y - (min_y - pad)) / span_y)
        px = int(nx * (args.width - 80) + 40)
        py = int((1.0 - ny) * (args.height - 80) + 40)
        return px, py

    i = 0
    real_t = 0.0
    last_real = pygame.time.get_ticks() / 1000.0
    run = True

    path_pts = []

    while run:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                run = False

        now = pygame.time.get_ticks() / 1000.0
        dt = now - last_real
        last_real = now
        real_t += dt * max(0.05, args.speed)

        while i + 1 < len(trace) and trace[i + 1][0] <= real_t:
            i += 1

        t, x, y, th = trace[i]
        px, py = to_px(x, y)
        path_pts.append((px, py))

        screen.fill(bg)

        if len(path_pts) > 1:
            pygame.draw.lines(screen, path_c, False, path_pts, 2)

        L, W = 0.35, 0.30
        corners = []
        for lx, ly in ((+L/2,+W/2),(+L/2,-W/2),(-L/2,-W/2),(-L/2,+W/2)):
            wx = x + lx*math.cos(th) - ly*math.sin(th)
            wy = y + lx*math.sin(th) + ly*math.cos(th)
            corners.append(to_px(wx, wy))

        pygame.draw.polygon(screen, bot_c, corners, width=2)
        hx = x + 0.23 * math.cos(th)
        hy = y + 0.23 * math.sin(th)
        pygame.draw.line(screen, head_c, (px, py), to_px(hx, hy), 3)

        txt = f"t={t:6.2f}s  idx={i+1}/{len(trace)}  speed={args.speed:.2f}x"
        screen.blit(font.render(txt, True, fg), (14, 12))

        pygame.display.flip()
        clock.tick(120)

        if i >= len(trace) - 1:
            # Hold last frame until window closed.
            pass

    pygame.quit()


if __name__ == "__main__":
    main()
