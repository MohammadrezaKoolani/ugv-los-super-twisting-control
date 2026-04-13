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
    Karl-style 3-DOF planar truck plant with lumped damping terms
    for practical heavy-truck simulation.

    State:
        x_n, y_n, psi, u, v_sway, r
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

    # Normalized controller outputs -> effective plant inputs
    tau_xc_clamped = max(-1.0, min(1.0, tau_xc))
    tau_psi_c_clamped = max(-1.0, min(1.0, tau_psi_c))

    F_x_max_sim = 85000.0     # [N]
    M_z_max_sim = 50000.0    # [N m]

    tau_x_eff = tau_xc_clamped * F_x_max_sim
    tau_psi_eff = tau_psi_c_clamped * M_z_max_sim

    # Lumped damping for practical simulation
    Y_v_sim = -25000.0       # [N / (m/s)]
    N_r_sim = -180000.0      # [N m / (rad/s)]

    # Karl-style control-oriented dynamics with lumped damping
    u_dot = (tau_x_eff + d_x + m * v * r + X_u_abs_u * u * abs(u)) / m
    v_dot = (Y_v_sim * v + d_y - m * u * r) / m
    r_dot = (tau_psi_eff + N_r_sim * r + d_psi) / I_z

    return UGVState(
        x_n=x_n + x_dot * dt,
        y_n=y_n + y_dot * dt,
        psi=wrap_angle(psi + psi_dot * dt),
        u=u + u_dot * dt,
        v_sway=v + v_dot * dt,
        r=r + r_dot * dt,
    )