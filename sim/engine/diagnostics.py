from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

from .auton import Segment


@dataclass
class _Sample:
    time_s: float
    theta_rad: float
    x_m: float
    y_m: float
    left_cmd: float
    right_cmd: float


class DiagnosticsTracker:
    def __init__(
        self,
        dt_s: float,
        cmd_threshold: float = 0.2,
        wheel_rps_threshold: float = 0.35,
        reverse_window_s: float = 0.15,
        turn_overshoot_issue_deg: float = 18.0,
        stop_drift_issue_m: float = 0.10,
    ):
        self.dt_s = dt_s
        self.cmd_threshold = cmd_threshold
        self.wheel_rps_threshold = wheel_rps_threshold
        self.reverse_window_s = reverse_window_s
        self.turn_overshoot_issue_deg = turn_overshoot_issue_deg
        self.stop_drift_issue_m = stop_drift_issue_m

        self._left_bad_steps = 0
        self._right_bad_steps = 0
        self._left_event_open = False
        self._right_event_open = False
        self._reverse_events: list[dict[str, float | str]] = []

        self._samples: list[_Sample] = []
        self._times: list[float] = []
        self._unwrapped_theta: list[float] = []

    def process(
        self,
        time_s: float,
        x_m: float,
        y_m: float,
        theta_rad: float,
        left_cmd: float,
        right_cmd: float,
        left_wheel_rps: float,
        right_wheel_rps: float,
    ) -> None:
        self._track_reverse(
            side="left",
            time_s=time_s,
            cmd=left_cmd,
            wheel_rps=left_wheel_rps,
        )
        self._track_reverse(
            side="right",
            time_s=time_s,
            cmd=right_cmd,
            wheel_rps=right_wheel_rps,
        )

        self._samples.append(
            _Sample(
                time_s=time_s,
                theta_rad=theta_rad,
                x_m=x_m,
                y_m=y_m,
                left_cmd=left_cmd,
                right_cmd=right_cmd,
            )
        )
        self._times.append(time_s)

        if not self._unwrapped_theta:
            self._unwrapped_theta.append(theta_rad)
        else:
            prev_raw = self._samples[-2].theta_rad
            raw_delta = theta_rad - prev_raw
            while raw_delta > math.pi:
                raw_delta -= 2.0 * math.pi
            while raw_delta < -math.pi:
                raw_delta += 2.0 * math.pi
            self._unwrapped_theta.append(self._unwrapped_theta[-1] + raw_delta)

    def finalize(self, scripted_segments: list[Segment] | None) -> dict[str, object]:
        reverse_summary = {
            "event_count": len(self._reverse_events),
            "events": self._reverse_events,
        }

        overshoot_summary = {
            "max_turn_overshoot_deg": 0.0,
            "max_stop_drift_m": 0.0,
            "max_stop_lateral_drift_m": 0.0,
            "turn_segments": [],
            "straight_stop_segments": [],
        }

        if scripted_segments and self._samples:
            overshoot_summary = self._analyze_scripted_segments(scripted_segments)

        issues: list[str] = []
        if reverse_summary["event_count"] > 0:
            issues.append("motor_direction_mismatch_detected")
        if float(overshoot_summary["max_turn_overshoot_deg"]) > self.turn_overshoot_issue_deg:
            issues.append("turn_overshoot_high")
        if float(overshoot_summary["max_stop_drift_m"]) > self.stop_drift_issue_m:
            issues.append("stop_drift_high")

        return {
            "reverse_motor_check": reverse_summary,
            "overshoot_check": overshoot_summary,
            "issues": issues,
            "issue_count": len(issues),
        }

    def _track_reverse(self, side: str, time_s: float, cmd: float, wheel_rps: float) -> None:
        cmd_active = abs(cmd) >= self.cmd_threshold
        wheel_active = abs(wheel_rps) >= self.wheel_rps_threshold
        mismatch = cmd_active and wheel_active and (cmd * wheel_rps < 0.0)

        if side == "left":
            if mismatch:
                self._left_bad_steps += 1
            else:
                self._left_bad_steps = 0
                self._left_event_open = False

            if (
                self._left_bad_steps * self.dt_s >= self.reverse_window_s
                and not self._left_event_open
            ):
                self._left_event_open = True
                self._reverse_events.append(
                    {
                        "side": "left",
                        "time_s": round(time_s, 4),
                        "reason": "cmd_sign_opposes_wheel_sign",
                    }
                )
        else:
            if mismatch:
                self._right_bad_steps += 1
            else:
                self._right_bad_steps = 0
                self._right_event_open = False

            if (
                self._right_bad_steps * self.dt_s >= self.reverse_window_s
                and not self._right_event_open
            ):
                self._right_event_open = True
                self._reverse_events.append(
                    {
                        "side": "right",
                        "time_s": round(time_s, 4),
                        "reason": "cmd_sign_opposes_wheel_sign",
                    }
                )

    def _analyze_scripted_segments(self, segments: list[Segment]) -> dict[str, object]:
        seg_ends: list[float] = []
        t = 0.0
        for seg in segments:
            t += seg.duration_s
            seg_ends.append(t)

        turn_segments: list[dict[str, float | int]] = []
        straight_stop_segments: list[dict[str, float | int]] = []

        max_turn_overshoot_deg = 0.0
        max_stop_drift_m = 0.0
        max_stop_lateral_drift_m = 0.0

        for i, seg in enumerate(segments):
            start_t = seg_ends[i - 1] if i > 0 else 0.0
            end_t = seg_ends[i]
            is_turn = seg.left_cmd * seg.right_cmd < -0.01

            if is_turn:
                window_end = min(end_t + 0.6, self._times[-1])
                heading_end = self._theta_at(end_t)
                peak_dev = self._peak_heading_deviation(end_t, window_end, heading_end)
                peak_deg = abs(math.degrees(peak_dev))
                max_turn_overshoot_deg = max(max_turn_overshoot_deg, peak_deg)
                turn_segments.append(
                    {
                        "segment_index": i,
                        "start_s": round(start_t, 4),
                        "end_s": round(end_t, 4),
                        "overshoot_deg": round(peak_deg, 4),
                    }
                )

            if i + 1 >= len(segments):
                continue

            next_seg = segments[i + 1]
            is_straight = abs(seg.left_cmd - seg.right_cmd) < 0.05 and abs(seg.left_cmd) > 0.2
            next_is_stop = abs(next_seg.left_cmd) < 0.05 and abs(next_seg.right_cmd) < 0.05

            if is_straight and next_is_stop:
                window_end = min(seg_ends[i + 1], end_t + 0.8, self._times[-1])
                drift_fwd, drift_lat = self._stop_drift(end_t, window_end)
                max_stop_drift_m = max(max_stop_drift_m, drift_fwd)
                max_stop_lateral_drift_m = max(max_stop_lateral_drift_m, drift_lat)
                straight_stop_segments.append(
                    {
                        "segment_index": i,
                        "start_s": round(start_t, 4),
                        "end_s": round(end_t, 4),
                        "forward_drift_m": round(drift_fwd, 5),
                        "lateral_drift_m": round(drift_lat, 5),
                    }
                )

        return {
            "max_turn_overshoot_deg": round(max_turn_overshoot_deg, 4),
            "max_stop_drift_m": round(max_stop_drift_m, 5),
            "max_stop_lateral_drift_m": round(max_stop_lateral_drift_m, 5),
            "turn_segments": turn_segments,
            "straight_stop_segments": straight_stop_segments,
        }

    def _idx_near_time(self, t: float) -> int:
        i = bisect.bisect_left(self._times, t)
        if i <= 0:
            return 0
        if i >= len(self._times):
            return len(self._times) - 1

        prev_i = i - 1
        if abs(self._times[prev_i] - t) <= abs(self._times[i] - t):
            return prev_i
        return i

    def _theta_at(self, t: float) -> float:
        return self._unwrapped_theta[self._idx_near_time(t)]

    def _peak_heading_deviation(self, t0: float, t1: float, base_theta: float) -> float:
        i0 = self._idx_near_time(t0)
        i1 = self._idx_near_time(t1)
        if i1 < i0:
            i0, i1 = i1, i0

        peak = 0.0
        for i in range(i0, i1 + 1):
            dev = self._unwrapped_theta[i] - base_theta
            if abs(dev) > abs(peak):
                peak = dev
        return peak

    def _stop_drift(self, t0: float, t1: float) -> tuple[float, float]:
        i0 = self._idx_near_time(t0)
        i1 = self._idx_near_time(t1)
        if i1 < i0:
            i0, i1 = i1, i0

        s0 = self._samples[i0]
        heading = self._unwrapped_theta[i0]
        c = math.cos(heading)
        s = math.sin(heading)

        max_forward = 0.0
        max_lateral = 0.0
        for i in range(i0, i1 + 1):
            si = self._samples[i]
            dx = si.x_m - s0.x_m
            dy = si.y_m - s0.y_m
            forward = dx * c + dy * s
            lateral = -dx * s + dy * c
            max_forward = max(max_forward, max(0.0, forward))
            max_lateral = max(max_lateral, abs(lateral))

        return max_forward, max_lateral
