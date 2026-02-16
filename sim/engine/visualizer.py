from __future__ import annotations

import math


class LiveVisualizer:
    """Optional matplotlib-based live viewer for robot state."""

    def __init__(self, robot_length_m: float = 0.35, robot_width_m: float = 0.30):
        self.robot_length_m = robot_length_m
        self.robot_width_m = robot_width_m
        self.enabled = False

        try:
            import matplotlib.pyplot as plt

            self.plt = plt
            self.fig, self.ax = plt.subplots(figsize=(7.5, 7.5))
            self.ax.set_title("VEX Sim Live View")
            self.ax.set_xlabel("x (m)")
            self.ax.set_ylabel("y (m)")
            self.ax.grid(True, alpha=0.25)
            self.ax.axis("equal")

            (self.path_line,) = self.ax.plot([], [], linewidth=1.5, label="path")
            (self.robot_line,) = self.ax.plot([], [], linewidth=2.2, label="robot")
            (self.head_line,) = self.ax.plot([], [], linewidth=2.0, label="heading")
            self.ax.legend(loc="upper right")

            self._path_x: list[float] = []
            self._path_y: list[float] = []
            self._last_draw_t = -1.0
            self.enabled = True
            plt.show(block=False)
        except Exception:
            self.enabled = False

    def update(self, time_s: float, x_m: float, y_m: float, theta_rad: float) -> None:
        if not self.enabled:
            return

        self._path_x.append(x_m)
        self._path_y.append(y_m)

        # Draw at ~25 FPS max to reduce overhead.
        if self._last_draw_t >= 0.0 and (time_s - self._last_draw_t) < 0.04:
            return
        self._last_draw_t = time_s

        corners = self._robot_outline(x_m, y_m, theta_rad)
        xs = [p[0] for p in corners] + [corners[0][0]]
        ys = [p[1] for p in corners] + [corners[0][1]]

        head_len = self.robot_length_m * 0.55
        hx = x_m + head_len * math.cos(theta_rad)
        hy = y_m + head_len * math.sin(theta_rad)

        self.path_line.set_data(self._path_x, self._path_y)
        self.robot_line.set_data(xs, ys)
        self.head_line.set_data([x_m, hx], [y_m, hy])

        self._auto_scale(x_m, y_m)
        self.plt.pause(0.001)

    def close(self) -> None:
        if not self.enabled:
            return
        self.plt.close(self.fig)

    def wait_until_closed(self) -> None:
        if not self.enabled:
            return
        self.plt.show(block=True)

    def _robot_outline(self, x: float, y: float, theta: float) -> list[tuple[float, float]]:
        hl = 0.5 * self.robot_length_m
        hw = 0.5 * self.robot_width_m

        local = [
            (+hl, +hw),
            (+hl, -hw),
            (-hl, -hw),
            (-hl, +hw),
        ]

        c = math.cos(theta)
        s = math.sin(theta)
        world: list[tuple[float, float]] = []
        for lx, ly in local:
            wx = x + lx * c - ly * s
            wy = y + lx * s + ly * c
            world.append((wx, wy))
        return world

    def _auto_scale(self, x: float, y: float) -> None:
        span = 1.6
        if self._path_x:
            min_x = min(self._path_x)
            max_x = max(self._path_x)
            min_y = min(self._path_y)
            max_y = max(self._path_y)
            span = max(span, max_x - min_x + 1.0, max_y - min_y + 1.0)

        self.ax.set_xlim(x - span / 2.0, x + span / 2.0)
        self.ax.set_ylim(y - span / 2.0, y + span / 2.0)
