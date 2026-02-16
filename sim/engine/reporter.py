from __future__ import annotations

import csv
import json
from pathlib import Path

from .types import SimStepOutput


class TraceWriter:
    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.trace_path = out_dir / "trace.csv"
        self.events_path = out_dir / "param_events.csv"

        self._trace_f = self.trace_path.open("w", newline="", encoding="utf-8")
        self._events_f = self.events_path.open("w", newline="", encoding="utf-8")
        self._trace = csv.writer(self._trace_f)
        self._events = csv.writer(self._events_f)

        self._trace.writerow(
            [
                "time_s",
                "x_m",
                "y_m",
                "theta_rad",
                "vx_mps",
                "vy_mps",
                "omega_rps",
                "left_cmd",
                "right_cmd",
                "left_wheel_rps",
                "right_wheel_rps",
                "battery_v",
                "imu_heading_deg",
                "rotation_left_deg",
                "rotation_right_deg",
                "slip_left_mps",
                "slip_right_mps",
            ]
        )
        self._events.writerow(["time_s", "event", "details"])

        self.max_speed_mps = 0.0
        self.max_slip_mps = 0.0
        self.last: SimStepOutput | None = None

    def write(self, out: SimStepOutput) -> None:
        speed = (out.state.vx_mps**2 + out.state.vy_mps**2) ** 0.5
        self.max_speed_mps = max(self.max_speed_mps, speed)
        self.max_slip_mps = max(self.max_slip_mps, out.slip_left_mps, out.slip_right_mps)
        self.last = out

        self._trace.writerow(
            [
                f"{out.time_s:.6f}",
                f"{out.state.x_m:.6f}",
                f"{out.state.y_m:.6f}",
                f"{out.state.theta_rad:.6f}",
                f"{out.state.vx_mps:.6f}",
                f"{out.state.vy_mps:.6f}",
                f"{out.state.omega_rps:.6f}",
                f"{out.left_cmd:.4f}",
                f"{out.right_cmd:.4f}",
                f"{out.left_wheel_rps:.6f}",
                f"{out.right_wheel_rps:.6f}",
                f"{out.battery_v:.4f}",
                f"{out.sensor.imu_heading_deg:.4f}",
                f"{out.sensor.rotation_left_deg:.4f}",
                f"{out.sensor.rotation_right_deg:.4f}",
                f"{out.slip_left_mps:.6f}",
                f"{out.slip_right_mps:.6f}",
            ]
        )

    def event(self, time_s: float, event: str, details: str) -> None:
        self._events.writerow([f"{time_s:.6f}", event, details])

    def close(self) -> None:
        self._trace_f.close()
        self._events_f.close()

    def write_report(
        self,
        out_dir: Path,
        duration_s: float,
        diagnostics: dict[str, object] | None = None,
    ) -> Path:
        report_path = out_dir / "report.json"
        last = self.last
        summary = {
            "duration_s": duration_s,
            "max_speed_mps": self.max_speed_mps,
            "max_slip_mps": self.max_slip_mps,
            "final_state": {
                "x_m": last.state.x_m if last else 0.0,
                "y_m": last.state.y_m if last else 0.0,
                "theta_rad": last.state.theta_rad if last else 0.0,
                "vx_mps": last.state.vx_mps if last else 0.0,
                "vy_mps": last.state.vy_mps if last else 0.0,
                "omega_rps": last.state.omega_rps if last else 0.0,
            },
            "diagnostics": diagnostics or {},
        }
        report_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return report_path
