from dataclasses import dataclass


@dataclass
class RobotState:
    x_m: float = 0.0
    y_m: float = 0.0
    theta_rad: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    omega_rps: float = 0.0
    left_wheel_rad: float = 0.0
    right_wheel_rad: float = 0.0


@dataclass
class MotorGroupState:
    cmd: float = 0.0
    wheel_rps: float = 0.0
    torque_nm: float = 0.0
    current_a: float = 0.0


@dataclass
class SensorFrame:
    imu_heading_deg: float
    rotation_left_deg: float
    rotation_right_deg: float


@dataclass
class SimStepOutput:
    time_s: float
    state: RobotState
    sensor: SensorFrame
    left_cmd: float
    right_cmd: float
    battery_v: float
    slip_left_mps: float
    slip_right_mps: float
