from __future__ import annotations

from dataclasses import dataclass
from random import Random


@dataclass
class MotorUnit:
    kv_scale: float
    kt_scale: float
    deadband: float


class MotorModel:
    def __init__(
        self,
        cartridge_rpm: float,
        stall_torque_nm: float,
        battery_nominal_v: float,
        mismatch_sigma_kv: float,
        mismatch_sigma_kt: float,
        deadband_sigma: float,
        rng: Random,
        count: int,
    ):
        self.free_speed_rad_s = cartridge_rpm * 2.0 * 3.141592653589793 / 60.0
        self.stall_torque_nm = stall_torque_nm
        self.battery_nominal_v = battery_nominal_v
        self.count = count

        self.kt_over_r = stall_torque_nm / battery_nominal_v
        self.ke = battery_nominal_v / self.free_speed_rad_s

        self.units: list[MotorUnit] = []
        for _ in range(count):
            kv = max(0.75, 1.0 + rng.gauss(0.0, mismatch_sigma_kv))
            kt = max(0.75, 1.0 + rng.gauss(0.0, mismatch_sigma_kt))
            db = max(0.0, abs(rng.gauss(0.0, deadband_sigma)))
            self.units.append(MotorUnit(kv_scale=kv, kt_scale=kt, deadband=db))

    @staticmethod
    def _apply_deadband(cmd: float, deadband: float) -> float:
        if abs(cmd) <= deadband:
            return 0.0
        sign = 1.0 if cmd > 0 else -1.0
        return sign * (abs(cmd) - deadband) / max(1e-6, (1.0 - deadband))

    def aggregate_torque_current(
        self,
        cmd: float,
        wheel_rad_s: float,
        battery_v: float,
    ) -> tuple[float, float]:
        total_torque = 0.0
        total_current = 0.0

        for unit in self.units:
            eff_cmd = self._apply_deadband(max(-1.0, min(1.0, cmd)), unit.deadband)
            v_cmd = eff_cmd * battery_v

            eff_ke = self.ke / unit.kv_scale
            current = (v_cmd - eff_ke * wheel_rad_s) / max(1e-6, battery_v / self.kt_over_r)
            torque = unit.kt_scale * self.kt_over_r * (v_cmd - eff_ke * wheel_rad_s)

            max_torque = unit.kt_scale * self.stall_torque_nm
            if torque > max_torque:
                torque = max_torque
            elif torque < -max_torque:
                torque = -max_torque

            total_torque += torque
            total_current += abs(current)

        return total_torque, total_current
