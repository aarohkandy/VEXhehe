from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass
class Segment:
    duration_s: float
    left_cmd: float
    right_cmd: float


class ScriptedAuton:
    def __init__(self, segments: list[Segment]):
        self.segments = segments
        self._seg_i = 0
        self._seg_t = 0.0

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
                )
            )
        return cls(segs)

    def command(self, dt_s: float) -> tuple[float, float]:
        if self._seg_i >= len(self.segments):
            return 0.0, 0.0

        seg = self.segments[self._seg_i]
        self._seg_t += dt_s
        if self._seg_t >= seg.duration_s:
            self._seg_i += 1
            self._seg_t = 0.0
        return seg.left_cmd, seg.right_cmd
