from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass
class RobotParams:
    mass_kg: float
    izz_kgm2: float
    wheel_radius_m: float
    track_width_m: float
    mu_long: float
    mu_lat: float
    k_long_n_per_mps: float
    k_lat_n_per_mps: float
    c_drag_n_per_mps: float
    cartridge_rpm: float
    stall_torque_nm: float
    motor_count_left: int
    motor_count_right: int
    battery_nominal_v: float
    battery_internal_r: float


@dataclass
class NoiseParams:
    seed: int
    imu_sigma_deg: float
    imu_bias_init_deg: float
    imu_bias_rw_deg_per_sqrt_s: float
    imu_latency_ms: float
    imu_dropout_rate: float
    rotation_sigma_deg: float
    rotation_quant_deg: float
    rotation_latency_ms: float
    rotation_dropout_rate: float


@dataclass
class FaultParams:
    motor_kv_sigma: float
    motor_kt_sigma: float
    motor_deadband_sigma: float


@dataclass
class RunParams:
    dt_s: float
    realtime: bool
    duration_s: float
    reload_period_s: float


class ConfigManager:
    def __init__(self, config_dir: Path):
        self.config_dir = config_dir
        self.paths = {
            "robot": config_dir / "robot.yaml",
            "noise": config_dir / "noise.yaml",
            "faults": config_dir / "faults.yaml",
            "run": config_dir / "run.yaml",
        }
        self._last_mtimes: dict[str, float] = {}
        self._last_check_s = 0.0

    def load_all(self) -> tuple[RobotParams, NoiseParams, FaultParams, RunParams]:
        robot_raw = self._load_yaml("robot")
        noise_raw = self._load_yaml("noise")
        faults_raw = self._load_yaml("faults")
        run_raw = self._load_yaml("run")
        return (
            RobotParams(**robot_raw),
            NoiseParams(**noise_raw),
            FaultParams(**faults_raw),
            RunParams(**run_raw),
        )

    def maybe_reload(
        self,
        now_s: float,
    ) -> tuple[bool, tuple[RobotParams, NoiseParams, FaultParams, RunParams] | None]:
        run_raw = self._load_yaml("run")
        reload_period_s = float(run_raw.get("reload_period_s", 0.25))
        if now_s - self._last_check_s < reload_period_s:
            return False, None

        self._last_check_s = now_s
        changed = False
        for key, path in self.paths.items():
            mtime = os.path.getmtime(path)
            old = self._last_mtimes.get(key)
            if old is None:
                self._last_mtimes[key] = mtime
                continue
            if mtime > old:
                self._last_mtimes[key] = mtime
                changed = True

        if not changed:
            return False, None

        return True, self.load_all()

    def _load_yaml(self, name: str) -> dict[str, Any]:
        path = self.paths[name]
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            raise ValueError(f"Config {path} is not a mapping")

        self._last_mtimes[name] = os.path.getmtime(path)
        return data
