from __future__ import annotations

import math

from ugv_control.models.states import HeadingControllerState, UGVState
from ugv_control.models.vehicle_base import VehicleParams
from ugv_control.types import HeadingControlOutput, LOSGuidanceOutput


def sign(value: float) -> float:
    """
    Sign function with sign(0) = 0.
    """
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def saturate(value: float, limit: float) -> float:
    """
    Symmetric saturation:
        sat_limit(value)
    """
    if limit <= 0.0:
        raise ValueError("limit must be positive.")

    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def wrap_angle(angle: float) -> float:
    """
    Wrap an angle to [-pi, pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def update_heading_controller(
    state: UGVState,
    guidance: LOSGuidanceOutput,
    controller_state: HeadingControllerState,
    params: VehicleParams,
    dt: float,
) -> tuple[HeadingControlOutput, HeadingControllerState]:
    """
    Update the super-twisting heading controller.

    Implements:
      - heading error and yaw-rate error from Eqs. (40)-(42)
      - sliding surface sigma_r used in Eq. (44)
      - steering controller in Eq. (44)

    Parameters
    ----------
    state : UGVState
        Current vehicle state.
    guidance : LOSGuidanceOutput
        Output from the LOS guidance law.
    controller_state : HeadingControllerState
        Internal heading controller state.
    params : VehicleParams
        Controller and vehicle parameters.
    dt : float
        Controller time step [s].

    Returns
    -------
    tuple[HeadingControlOutput, HeadingControllerState]
        The controller output and the updated controller state.
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    # Heading tracking error: psi_tilde = psi - psi_d
    psi_tilde = wrap_angle(state.psi - guidance.psi_d)

    # Yaw-rate tracking error: r_tilde = r - r_d
    r_tilde = state.r - guidance.r_d

    # Sliding surface used in Eq. (44)
    sigma_r = psi_tilde + params.k_r * r_tilde

    # Eq. (44): raw steering command before saturation
    raw_tau_psi_c = (
        -params.k_psi * math.sqrt(abs(sigma_r)) * sign(sigma_r)
        + controller_state.tau_psi1
    )

    # Eq. (44): saturated steering command
    tau_psi_c = saturate(raw_tau_psi_c, params.tau_psi_max)

    # Eq. (44): internal state derivative
    if abs(tau_psi_c) >= params.tau_psi_max:
        tau_psi1_dot = 0.0
    else:
        tau_psi1_dot = -params.k_psi1 * sign(sigma_r)

    # Discrete-time integration of tau_psi1
    new_tau_psi1 = controller_state.tau_psi1 + tau_psi1_dot * dt

    new_state = HeadingControllerState(tau_psi1=new_tau_psi1)
    output = HeadingControlOutput(
        tau_psi_c=tau_psi_c,
        psi_tilde=psi_tilde,
        r_tilde=r_tilde,
        sigma_r=sigma_r,
    )

    return output, new_state