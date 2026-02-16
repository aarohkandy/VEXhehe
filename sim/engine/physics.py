from __future__ import annotations

import math
from dataclasses import dataclass

from .config import RobotParams
from .types import RobotState


@dataclass
class SideForce:
    fx_n: float
    fy_n: float
    slip_mps: float


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class DynamicsEngine:
    def __init__(self, params: RobotParams):
        self.params = params

    def update_params(self, params: RobotParams) -> None:
        self.params = params

    def step(
        self,
        state: RobotState,
        dt_s: float,
        left_torque_nm: float,
        right_torque_nm: float,
    ) -> tuple[RobotState, float, float]:
        p = self.params
        g = 9.80665

        # Contact point velocities in body frame for left/right wheel lines.
        y_l = +0.5 * p.track_width_m
        y_r = -0.5 * p.track_width_m

        v_lx = state.vx_mps - state.omega_rps * y_l
        v_ly = state.vy_mps
        v_rx = state.vx_mps - state.omega_rps * y_r
        v_ry = state.vy_mps

        # Motor torque to wheel longitudinal force.
        f_drive_l = left_torque_nm / max(1e-6, p.wheel_radius_m)
        f_drive_r = right_torque_nm / max(1e-6, p.wheel_radius_m)

        n_side = 0.5 * p.mass_kg * g
        max_fx = p.mu_long * n_side
        max_fy = p.mu_lat * n_side

        left = self._side_force(f_drive_l, v_lx, v_ly, max_fx, max_fy, p)
        right = self._side_force(f_drive_r, v_rx, v_ry, max_fx, max_fy, p)

        fx_body = left.fx_n + right.fx_n - p.c_drag_n_per_mps * state.vx_mps
        fy_body = left.fy_n + right.fy_n - p.c_drag_n_per_mps * state.vy_mps

        tau_z = -y_l * left.fx_n - y_r * right.fx_n

        ax = fx_body / p.mass_kg
        ay = fy_body / p.mass_kg
        alpha = tau_z / p.izz_kgm2

        # Semi-implicit integration for better stability.
        vx = state.vx_mps + ax * dt_s
        vy = state.vy_mps + ay * dt_s
        omega = state.omega_rps + alpha * dt_s

        theta = state.theta_rad + omega * dt_s

        c = math.cos(theta)
        s = math.sin(theta)
        vx_world = c * vx - s * vy
        vy_world = s * vx + c * vy

        x = state.x_m + vx_world * dt_s
        y = state.y_m + vy_world * dt_s

        left_wheel_rad = state.left_wheel_rad + (v_lx / max(1e-6, p.wheel_radius_m)) * dt_s
        right_wheel_rad = state.right_wheel_rad + (v_rx / max(1e-6, p.wheel_radius_m)) * dt_s

        return (
            RobotState(
                x_m=x,
                y_m=y,
                theta_rad=theta,
                vx_mps=vx,
                vy_mps=vy,
                omega_rps=omega,
                left_wheel_rad=left_wheel_rad,
                right_wheel_rad=right_wheel_rad,
            ),
            left.slip_mps,
            right.slip_mps,
        )

    @staticmethod
    def _side_force(
        f_drive_x: float,
        v_x: float,
        v_y: float,
        max_fx: float,
        max_fy: float,
        p: RobotParams,
    ) -> SideForce:
        # Orthotropic omni model:
        # longitudinal: motor drive minus slip damping + viscous term
        # lateral: omni wheel allows easier side slip (smaller saturation)
        f_long = f_drive_x - p.k_long_n_per_mps * v_x
        f_lat = -p.k_lat_n_per_mps * v_y

        fx = _clamp(f_long, -max_fx, max_fx)
        fy = _clamp(f_lat, -max_fy, max_fy)
        slip = abs(v_y) + max(0.0, abs(v_x) - abs(fx) / max(1e-6, p.k_long_n_per_mps))
        return SideForce(fx_n=fx, fy_n=fy, slip_mps=slip)
