from __future__ import annotations

import math

from ugv_control.interfaces.eight_wheel_mapper import EightWheelCommand
from ugv_control.models.states import UGVState
from ugv_control.models.vehicle_base import VehicleParams


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _dual_front_ratio(params: VehicleParams) -> float:
    denom = (2.0 * params.L1 - params.L3 - params.L4)
    if abs(denom) < 1e-9:
        return 1.0
    return (2.0 * params.L2 - params.L3 - params.L4) / denom


def _equivalent_max_yaw_moment(params: VehicleParams) -> float:
    """
    Convert the available front-steering authority into an equivalent
    maximum yaw-moment-like control level for the lumped Karl-style plant.
    """
    a_s = _dual_front_ratio(params)

    delta1_max = params.max_steer_axle1
    delta2_max = clamp(a_s * delta1_max, -params.max_steer_axle2, params.max_steer_axle2)

    front_denom = max(params.L1 + params.L2, 1e-6)
    delta_eff_max = (params.L1 * delta1_max + params.L2 * delta2_max) / front_denom

    L_front = 0.5 * (params.L1 + params.L2)
    L_rear = 0.5 * (abs(params.L3) + abs(params.L4))
    L_eq = max(L_front + L_rear, 1e-6)

    u_ref = max(params.desired_speed, 1.0)
    r_ref_max = u_ref * math.tan(delta_eff_max) / L_eq

    return params.I_z * r_ref_max / max(params.yaw_time_constant, 1e-6)


def simulate_step_truck_karl_style(
    state: UGVState,
    tau_xc: float,
    tau_psi_c: float,
    params: VehicleParams,
    dt: float,
    d_x: float = 0.0,
    d_y: float = 0.0,
    d_psi: float = 0.0,
    command: EightWheelCommand | None = None,
) -> UGVState:
    """
    Tire-stiffness-free, Karl-consistent, control-oriented truck plant.

    This is not the LPV tire model from the 2025 truck paper.
    It is a lumped plant intended to stay compatible with Karl-style
    LOS + super-twisting control while preserving the 8-wheel command interface.
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    m = params.m
    I_z = params.I_z
    X_u_abs_u = params.X_u_abs_u

    x_n = state.x_n
    y_n = state.y_n
    psi = state.psi
    u = state.u
    v = state.v_sway
    r = state.r

    # ------------------------------------------------------------
    # Longitudinal force from driven-wheel torques
    # ------------------------------------------------------------
    if command is not None:
        total_drive_torque = (
            command.torque_axle3_left
            + command.torque_axle3_right
            + command.torque_axle4_left
            + command.torque_axle4_right
        )
    else:
        drive_norm = 0.0
        if params.tau_x_max > 0.0:
            drive_norm = tau_xc / params.tau_x_max
        drive_norm = clamp(drive_norm, -1.0, 1.0)

        total_drive_torque = (
            drive_norm
            * params.max_drive_torque_per_wheel
            * max(params.num_driven_wheels, 1)
        )

    F_x = total_drive_torque / max(params.wheel_radius, 1e-6)

    # ------------------------------------------------------------
    # Yaw control channel as a lumped generalized input
    # ------------------------------------------------------------
    steer_norm = 0.0
    if params.tau_psi_max > 0.0:
        steer_norm = tau_psi_c / params.tau_psi_max
    steer_norm = clamp(steer_norm, -1.0, 1.0)

    N_psi = steer_norm * _equivalent_max_yaw_moment(params)

    # ------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------
    x_dot = u * math.cos(psi) - v * math.sin(psi)
    y_dot = u * math.sin(psi) + v * math.cos(psi)
    psi_dot = r

    # ------------------------------------------------------------
    # Karl-style lumped dynamics with mild damping
    # ------------------------------------------------------------
    u_dot = (F_x + d_x + m * v * r + X_u_abs_u * u * abs(u)) / m
    v_dot = (-m * u * r - (m / max(params.sway_time_constant, 1e-6)) * v + d_y) / m
    r_dot = (N_psi - (I_z / max(params.yaw_time_constant, 1e-6)) * r + d_psi) / I_z

    return UGVState(
        x_n=x_n + x_dot * dt,
        y_n=y_n + y_dot * dt,
        psi=wrap_angle(psi + psi_dot * dt),
        u=u + u_dot * dt,
        v_sway=v + v_dot * dt,
        r=r + r_dot * dt,
    )