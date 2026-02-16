from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from random import Random

from .config import NoiseParams
from .types import RobotState, SensorFrame


@dataclass
class DelayedValue:
    release_time_s: float
    value: float


class SensorModel:
    def __init__(self, params: NoiseParams, rng: Random):
        self.params = params
        self.rng = rng
        self.imu_bias_deg = params.imu_bias_init_deg
        self.imu_queue: deque[DelayedValue] = deque()
        self.rot_l_queue: deque[DelayedValue] = deque()
        self.rot_r_queue: deque[DelayedValue] = deque()

    def update_params(self, params: NoiseParams) -> None:
        self.params = params

    def sample(self, now_s: float, state: RobotState) -> SensorFrame:
        p = self.params

        self.imu_bias_deg += self.rng.gauss(0.0, p.imu_bias_rw_deg_per_sqrt_s) * math.sqrt(max(1e-6, 0.002))
        heading_deg = math.degrees(state.theta_rad) + self.imu_bias_deg + self.rng.gauss(0.0, p.imu_sigma_deg)

        rot_l_deg = math.degrees(state.left_wheel_rad) + self.rng.gauss(0.0, p.rotation_sigma_deg)
        rot_r_deg = math.degrees(state.right_wheel_rad) + self.rng.gauss(0.0, p.rotation_sigma_deg)

        rot_l_deg = self._quantize(rot_l_deg, p.rotation_quant_deg)
        rot_r_deg = self._quantize(rot_r_deg, p.rotation_quant_deg)

        if self.rng.random() < p.imu_dropout_rate:
            heading_deg = self._last_or_default(self.imu_queue, 0.0)
        if self.rng.random() < p.rotation_dropout_rate:
            rot_l_deg = self._last_or_default(self.rot_l_queue, 0.0)
        if self.rng.random() < p.rotation_dropout_rate:
            rot_r_deg = self._last_or_default(self.rot_r_queue, 0.0)

        self.imu_queue.append(DelayedValue(now_s + p.imu_latency_ms / 1000.0, heading_deg))
        self.rot_l_queue.append(DelayedValue(now_s + p.rotation_latency_ms / 1000.0, rot_l_deg))
        self.rot_r_queue.append(DelayedValue(now_s + p.rotation_latency_ms / 1000.0, rot_r_deg))

        out_imu = self._drain(self.imu_queue, now_s)
        out_l = self._drain(self.rot_l_queue, now_s)
        out_r = self._drain(self.rot_r_queue, now_s)

        return SensorFrame(
            imu_heading_deg=out_imu,
            rotation_left_deg=out_l,
            rotation_right_deg=out_r,
        )

    @staticmethod
    def _quantize(v: float, q: float) -> float:
        if q <= 0:
            return v
        return round(v / q) * q

    @staticmethod
    def _last_or_default(q: deque[DelayedValue], default: float) -> float:
        if not q:
            return default
        return q[-1].value

    @staticmethod
    def _drain(q: deque[DelayedValue], now_s: float) -> float:
        if not q:
            return 0.0

        out = q[0].value
        while q and q[0].release_time_s <= now_s:
            out = q.popleft().value
        return out
