from __future__ import annotations

from dataclasses import dataclass

from ugv_control.models.vehicle_base import VehicleParams
from ugv_control.types import ControlOutput


@dataclass(frozen=True, slots=True)
class EightWheelCommand:
    steer_axle1: float
    steer_axle2: float

    torque_axle3_left: float
    torque_axle3_right: float
    torque_axle4_left: float
    torque_axle4_right: float


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def compute_dual_front_steering_ratio(params: VehicleParams) -> float:
    denom = (2.0 * params.L1 - params.L3 - params.L4)
    if abs(denom) < 1e-9:
        raise ValueError("Invalid axle geometry: steering ratio denominator is zero.")
    return (2.0 * params.L2 - params.L3 - params.L4) / denom


def map_control_to_eight_wheel(
    control: ControlOutput,
    params: VehicleParams,
) -> EightWheelCommand:
    """
    Map Karl-style control outputs:
        tau_xc, tau_psi_c

    into real 8-wheel truck actuator commands.
    """

    # -------------------------
    # 1) Steering mapping
    # -------------------------
    a_s = compute_dual_front_steering_ratio(params)

    # Interpret tau_psi_c as normalized steering effort in [-tau_psi_max, tau_psi_max]
    steer_norm = 0.0
    if params.tau_psi_max > 0.0:
        steer_norm = control.tau_psi_c / params.tau_psi_max

    steer_norm = clamp(steer_norm, -1.0, 1.0)

    delta1 = steer_norm * params.max_steer_axle1
    delta2 = a_s * delta1

    delta2 = clamp(delta2, -params.max_steer_axle2, params.max_steer_axle2)

    # -------------------------
    # 2) Drive torque mapping
    # -------------------------
    # Interpret tau_xc as normalized drive effort
    drive_norm = 0.0
    if params.tau_x_max > 0.0:
        drive_norm = control.tau_xc / params.tau_x_max

    # If no reverse is allowed:
    drive_norm = clamp(drive_norm, -1.0, 1.0)

    #drive_torque = drive_norm * params.max_drive_torque_per_wheel

    total_drive_torque = drive_norm * params.max_drive_torque_per_wheel * params.num_driven_wheels
    per_wheel_torque = total_drive_torque / max(params.num_driven_wheels, 1)

    return EightWheelCommand(
        steer_axle1=delta1,
        steer_axle2=delta2,
        torque_axle3_left=per_wheel_torque,
        torque_axle3_right=per_wheel_torque,
        torque_axle4_left=per_wheel_torque,
        torque_axle4_right=per_wheel_torque,
    )