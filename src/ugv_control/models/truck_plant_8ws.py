from __future__ import annotations

import math

from ugv_control.models.states import UGVState
from ugv_control.models.vehicle_base import VehicleParams


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def simulate_step_truck_karl_style(
    state: UGVState,
    tau_xc: float,
    tau_psi_c: float,
    params: VehicleParams,
    dt: float,
    d_x: float = 0.0,
    d_y: float = 0.0,
    d_psi: float = 0.0,
) -> UGVState:
    """
    Karl-style 3-DOF planar truck plant.

    State:
        x_n, y_n, psi, u, v_sway, r

    Inputs:
        tau_xc, tau_psi_c

    Model style:
        eta_dot = R(psi) * nu
        M nu_dot + C(nu) nu + D(nu) nu = tau + d
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

    # Kinematics
    x_dot = u * math.cos(psi) - v * math.sin(psi)
    y_dot = u * math.sin(psi) + v * math.cos(psi)
    psi_dot = r

    # Karl-style control-oriented dynamics
    u_dot = (tau_xc + d_x + m * v * r + X_u_abs_u * u * abs(u)) / m
    v_dot = (d_y - m * u * r) / m
    r_dot = (tau_psi_c + d_psi) / I_z

    return UGVState(
        x_n=x_n + x_dot * dt,
        y_n=y_n + y_dot * dt,
        psi=wrap_angle(psi + psi_dot * dt),
        u=u + u_dot * dt,
        v_sway=v + v_dot * dt,
        r=r + r_dot * dt,
    )