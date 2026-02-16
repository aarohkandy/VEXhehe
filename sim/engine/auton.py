from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .types import RobotState


@dataclass
class Segment:
    duration_s: float
    left_cmd: float
    right_cmd: float
    label: str = ""


class ScriptedAuton:
    def __init__(self, segments: list[Segment]):
        self.segments = segments
        self._seg_i = 0
        self._seg_t = 0.0
        self._last_label = "IDLE"

    @classmethod
    def from_yaml(cls, path: Path) -> "ScriptedAuton":
        with path.open("r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        if not isinstance(raw, dict) or "segments" not in raw:
            raise ValueError(f"Invalid scenario file: {path}")

        segs: list[Segment] = []
        for item in raw["segments"]:
            segs.append(
                Segment(
                    duration_s=float(item["duration_s"]),
                    left_cmd=float(item["left_cmd"]),
                    right_cmd=float(item["right_cmd"]),
                    label=str(item.get("label", "")),
                )
            )
        return cls(segs)

    def command(self, dt_s: float) -> tuple[float, float]:
        if self._seg_i >= len(self.segments):
            self._last_label = "DONE"
            return 0.0, 0.0

        seg = self.segments[self._seg_i]
        self._last_label = seg.label or f"SEGMENT_{self._seg_i + 1}"
        self._seg_t += dt_s
        if self._seg_t >= seg.duration_s:
            self._seg_i += 1
            self._seg_t = 0.0
        return seg.left_cmd, seg.right_cmd

    def current_index(self) -> int:
        return min(self._seg_i + 1, len(self.segments))

    def segment_count(self) -> int:
        return len(self.segments)

    def current_label(self) -> str:
        return self._last_label


@dataclass
class Waypoint:
    x_m: float
    y_m: float


class WaypointPathAuton:
    """State-feedback waypoint follower for curved and straight paths."""

    def __init__(
        self,
        waypoints: list[Waypoint],
        max_cmd: float,
        min_cmd: float,
        k_dist: float,
        k_heading: float,
        reach_tolerance_m: float,
        stop_tolerance_m: float,
    ):
        if len(waypoints) < 2:
            raise ValueError("waypoint path requires at least two waypoints")
        self.waypoints = waypoints
        self.max_cmd = max(0.1, min(1.0, max_cmd))
        self.min_cmd = max(0.0, min(self.max_cmd, min_cmd))
        self.k_dist = max(0.0, k_dist)
        self.k_heading = max(0.0, k_heading)
        self.reach_tolerance_m = max(0.01, reach_tolerance_m)
        self.stop_tolerance_m = max(0.01, stop_tolerance_m)
        self.target_i = 1
        self.finished = False

    @classmethod
    def from_yaml(cls, path: Path) -> "WaypointPathAuton":
        with path.open("r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)

        if not isinstance(raw, dict) or "waypoints" not in raw:
            raise ValueError(f"Invalid waypoint scenario file: {path}")
        waypoints_raw = raw.get("waypoints")
        if not isinstance(waypoints_raw, list):
            raise ValueError("waypoints must be a list")

        waypoints: list[Waypoint] = []
        for p in waypoints_raw:
            if not isinstance(p, dict):
                raise ValueError("waypoint entry must be a mapping")
            waypoints.append(Waypoint(x_m=float(p["x_m"]), y_m=float(p["y_m"])))

        ctrl = raw.get("controller", {})
        if not isinstance(ctrl, dict):
            ctrl = {}

        return cls(
            waypoints=waypoints,
            max_cmd=float(ctrl.get("max_cmd", 0.9)),
            min_cmd=float(ctrl.get("min_cmd", 0.15)),
            k_dist=float(ctrl.get("k_dist", 1.0)),
            k_heading=float(ctrl.get("k_heading", 1.6)),
            reach_tolerance_m=float(ctrl.get("reach_tolerance_m", 0.10)),
            stop_tolerance_m=float(ctrl.get("stop_tolerance_m", 0.05)),
        )

    def command(self, _dt_s: float, state: RobotState) -> tuple[float, float]:
        if self.finished:
            return 0.0, 0.0

        target = self.waypoints[self.target_i]
        dx = target.x_m - state.x_m
        dy = target.y_m - state.y_m
        dist = math.hypot(dx, dy)

        # Move through intermediate points once we're close enough.
        if self.target_i < len(self.waypoints) - 1 and dist < self.reach_tolerance_m:
            self.target_i += 1
            target = self.waypoints[self.target_i]
            dx = target.x_m - state.x_m
            dy = target.y_m - state.y_m
            dist = math.hypot(dx, dy)

        if self.target_i == len(self.waypoints) - 1 and dist < self.stop_tolerance_m:
            self.finished = True
            return 0.0, 0.0

        desired_heading = math.atan2(dy, dx)
        heading_err = self._wrap(desired_heading - state.theta_rad)

        # Distance controls forward speed; heading controls curvature.
        forward = self.k_dist * dist
        forward = max(self.min_cmd, min(self.max_cmd, forward))
        turn = self.k_heading * heading_err
        turn = max(-self.max_cmd, min(self.max_cmd, turn))

        left = max(-1.0, min(1.0, forward - turn))
        right = max(-1.0, min(1.0, forward + turn))
        return left, right

    def target_index(self) -> int:
        return self.target_i

    def waypoint_count(self) -> int:
        return len(self.waypoints)

    def current_target(self) -> tuple[float, float]:
        t = self.waypoints[self.target_i]
        return t.x_m, t.y_m

    @staticmethod
    def _wrap(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a
