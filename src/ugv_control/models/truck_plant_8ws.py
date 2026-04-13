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
    Heavy-truck plant without tire stiffnesses.

    This keeps the Karl-style 3-DOF state:
        x_n, y_n, psi, u, v_sway, r

    but uses an actuator-oriented truck interpretation:
      - dual front steering angles
      - rear wheel torques

    The model is intentionally control-oriented:
      - surge comes from summed wheel torques and quadratic drag
      - yaw comes from an effective steering angle and first-order yaw response
      - sway is lumped as a first-order response tied to steering
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

    # Equivalent front / rear geometry for a lumped heavy-truck model
    L_front = 0.5 * (params.L1 + params.L2)
    L_rear = 0.5 * (abs(params.L3) + abs(params.L4))
    L_eq = max(L_front + L_rear, 1e-6)

    wheel_radius = params.wheel_radius
    yaw_time_constant = params.yaw_time_constant
    sway_time_constant = params.sway_time_constant
    beta_gain = params.beta_gain

    # ------------------------------------------------------------
    # Interpret actuator commands
    # ------------------------------------------------------------
    if command is not None:
        delta1 = command.steer_axle1
        delta2 = command.steer_axle2

        total_drive_torque = (
            command.torque_axle3_left
            + command.torque_axle3_right
            + command.torque_axle4_left
            + command.torque_axle4_right
        )
    else:
        # Backward-compatible fallback:
        # interpret high-level controller outputs as normalized truck commands
        steer_norm = 0.0
        if params.tau_psi_max > 0.0:
            steer_norm = tau_psi_c / params.tau_psi_max
        steer_norm = clamp(steer_norm, -1.0, 1.0)

        delta1 = steer_norm * params.max_steer_axle1
        delta2 = _dual_front_ratio(params) * delta1
        delta2 = clamp(delta2, -params.max_steer_axle2, params.max_steer_axle2)

        drive_norm = 0.0
        if params.tau_x_max > 0.0:
            drive_norm = tau_xc / params.tau_x_max
        drive_norm = clamp(drive_norm, -1.0, 1.0)

        total_drive_torque = (
            drive_norm
            * params.max_drive_torque_per_wheel
            * max(params.num_driven_wheels, 1)
        )

    # Effective dual-front steering angle
    front_denom = max(params.L1 + params.L2, 1e-6)
    delta_eff = (params.L1 * delta1 + params.L2 * delta2) / front_denom
    delta_eff = clamp(delta_eff, -1.2, 1.2)

    # Drive torque -> total longitudinal force
    F_x = total_drive_torque / max(wheel_radius, 1e-6)

    # ------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------
    x_dot = u * math.cos(psi) - v * math.sin(psi)
    y_dot = u * math.sin(psi) + v * math.cos(psi)
    psi_dot = r

    # ------------------------------------------------------------
    # Lumped dynamics
    # ------------------------------------------------------------
    # Surge
    u_dot = (F_x + d_x + m * v * r + X_u_abs_u * u * abs(u)) / m

    # Yaw: first-order response toward kinematic yaw-rate command
    if abs(math.cos(delta_eff)) < 1e-6:
        r_cmd = 0.0
    else:
        r_cmd = u * math.tan(delta_eff) / L_eq

    r_dot = (r_cmd - r) / max(yaw_time_constant, 1e-6) + d_psi / I_z

    # Sway: lumped side-slip response without tire stiffnesses
    beta_ref = math.atan(beta_gain * (L_rear / L_eq) * math.tan(delta_eff))
    v_ref = u * math.sin(beta_ref)

    v_dot = (v_ref - v) / max(sway_time_constant, 1e-6) + d_y / m

    return UGVState(
        x_n=x_n + x_dot * dt,
        y_n=y_n + y_dot * dt,
        psi=wrap_angle(psi + psi_dot * dt),
        u=u + u_dot * dt,
        v_sway=v + v_dot * dt,
        r=r + r_dot * dt,
    )