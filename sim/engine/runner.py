from __future__ import annotations

import argparse
import importlib
import json
import time
from dataclasses import dataclass
from pathlib import Path
from random import Random

from .auton import ScriptedAuton
from .config import ConfigManager
from .motor import MotorModel
from .physics import DynamicsEngine
from .reporter import TraceWriter
from .sensors import SensorModel
from .types import MotorGroupState, RobotState, SimStepOutput


@dataclass
class ProsControlBuffer:
    left_cmd: float = 0.0
    right_cmd: float = 0.0


class ProsAdapter:
    def __init__(self, module_name: str, buffer: ProsControlBuffer):
        self.module_name = module_name
        self.buffer = buffer
        self._task = None

    def start(self) -> None:
        mod = importlib.import_module(self.module_name)
        if not hasattr(mod, "autonomous"):
            raise ValueError(f"PROS module {self.module_name} must define autonomous(api)")
        api = _ProsApi(self.buffer)
        self._task = mod.autonomous(api)

    def command(self, dt_s: float) -> tuple[float, float]:
        if self._task is None:
            self.start()
        # Run the user program as a generator. Yielded values are treated as delays.
        try:
            next(self._task)
        except StopIteration:
            pass
        return self.buffer.left_cmd, self.buffer.right_cmd


class _ProsApi:
    def __init__(self, buffer: ProsControlBuffer):
        self.buffer = buffer

    def set_tank(self, left: float, right: float) -> None:
        self.buffer.left_cmd = max(-1.0, min(1.0, left))
        self.buffer.right_cmd = max(-1.0, min(1.0, right))

    def delay(self, _ms: int):
        # Sim-side cooperative yield.
        yield


def _battery_voltage(nominal: float, internal_r: float, total_current: float) -> float:
    return max(7.0, nominal - internal_r * total_current)


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Headless VEX simulator")
    p.add_argument("--config-dir", default="config", type=str)
    p.add_argument("--scenario", default="scenarios/default_auton.yaml", type=str)
    p.add_argument("--pros-module", default=None, type=str)
    p.add_argument("--out", default="output/run", type=str)
    p.add_argument("--duration", default=None, type=float)
    p.add_argument("--realtime", action="store_true", default=True)
    p.add_argument("--no-realtime", action="store_true")
    return p


def main() -> None:
    args = _build_arg_parser().parse_args()
    root = Path(__file__).resolve().parents[1]
    config_dir = (root / args.config_dir).resolve()

    cfg = ConfigManager(config_dir)
    robot, noise, faults, run = cfg.load_all()
    if args.duration is not None:
        run.duration_s = args.duration
    if args.no_realtime:
        run.realtime = False
    elif args.realtime:
        run.realtime = True

    rng = Random(noise.seed)
    dyn = DynamicsEngine(robot)
    sensors = SensorModel(noise, rng)

    left_model = MotorModel(
        cartridge_rpm=robot.cartridge_rpm,
        stall_torque_nm=robot.stall_torque_nm,
        battery_nominal_v=robot.battery_nominal_v,
        mismatch_sigma_kv=faults.motor_kv_sigma,
        mismatch_sigma_kt=faults.motor_kt_sigma,
        deadband_sigma=faults.motor_deadband_sigma,
        rng=rng,
        count=robot.motor_count_left,
    )
    right_model = MotorModel(
        cartridge_rpm=robot.cartridge_rpm,
        stall_torque_nm=robot.stall_torque_nm,
        battery_nominal_v=robot.battery_nominal_v,
        mismatch_sigma_kv=faults.motor_kv_sigma,
        mismatch_sigma_kt=faults.motor_kt_sigma,
        deadband_sigma=faults.motor_deadband_sigma,
        rng=rng,
        count=robot.motor_count_right,
    )

    if args.pros_module:
        buffer = ProsControlBuffer()
        controller = ProsAdapter(args.pros_module, buffer)
    else:
        controller = ScriptedAuton.from_yaml((root / args.scenario).resolve())

    out_dir = (root / args.out).resolve()
    writer = TraceWriter(out_dir)

    state = RobotState()
    left_group = MotorGroupState()
    right_group = MotorGroupState()

    t = 0.0
    sim_start = time.perf_counter()
    last_reload_check = 0.0

    while t < run.duration_s:
        wall_loop_start = time.perf_counter()

        left_cmd, right_cmd = controller.command(run.dt_s)
        left_group.cmd = left_cmd
        right_group.cmd = right_cmd

        left_wheel_rps = (state.vx_mps - state.omega_rps * (robot.track_width_m / 2.0)) / max(1e-6, robot.wheel_radius_m) / (2.0 * 3.141592653589793)
        right_wheel_rps = (state.vx_mps + state.omega_rps * (robot.track_width_m / 2.0)) / max(1e-6, robot.wheel_radius_m) / (2.0 * 3.141592653589793)

        left_group.wheel_rps = left_wheel_rps
        right_group.wheel_rps = right_wheel_rps

        battery_v_guess = robot.battery_nominal_v
        lt, li = left_model.aggregate_torque_current(left_group.cmd, left_wheel_rps * 2.0 * 3.141592653589793, battery_v_guess)
        rt, ri = right_model.aggregate_torque_current(right_group.cmd, right_wheel_rps * 2.0 * 3.141592653589793, battery_v_guess)
        battery_v = _battery_voltage(robot.battery_nominal_v, robot.battery_internal_r, li + ri)
        lt, li = left_model.aggregate_torque_current(left_group.cmd, left_wheel_rps * 2.0 * 3.141592653589793, battery_v)
        rt, ri = right_model.aggregate_torque_current(right_group.cmd, right_wheel_rps * 2.0 * 3.141592653589793, battery_v)

        left_group.torque_nm = lt
        right_group.torque_nm = rt
        left_group.current_a = li
        right_group.current_a = ri

        state, slip_left, slip_right = dyn.step(state, run.dt_s, lt, rt)
        sensor = sensors.sample(t, state)

        writer.write(
            SimStepOutput(
                time_s=t,
                state=state,
                sensor=sensor,
                left_cmd=left_cmd,
                right_cmd=right_cmd,
                battery_v=battery_v,
                slip_left_mps=slip_left,
                slip_right_mps=slip_right,
            )
        )

        if t - last_reload_check >= run.reload_period_s:
            last_reload_check = t
            changed, new_cfg = cfg.maybe_reload(t)
            if changed and new_cfg is not None:
                robot, noise, faults, run = new_cfg
                dyn.update_params(robot)
                sensors.update_params(noise)
                writer.event(
                    t,
                    "hot_reload",
                    json.dumps(
                        {
                            "mass_kg": robot.mass_kg,
                            "mu_long": robot.mu_long,
                            "mu_lat": robot.mu_lat,
                            "imu_sigma_deg": noise.imu_sigma_deg,
                        }
                    ),
                )

        t += run.dt_s
        if run.realtime:
            elapsed = time.perf_counter() - wall_loop_start
            sleep_s = run.dt_s - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    writer.close()
    report_path = writer.write_report(out_dir, t)
    sim_elapsed = time.perf_counter() - sim_start

    print(f"Simulation complete in {sim_elapsed:.3f}s (sim time {t:.3f}s)")
    print(f"Trace: {writer.trace_path}")
    print(f"Events: {writer.events_path}")
    print(f"Report: {report_path}")
