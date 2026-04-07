from __future__ import annotations

import math

from ugv_control.models.states import SurgeControllerState, UGVState
from ugv_control.models.vehicle_base import VehicleParams
from ugv_control.types import SurgeControlOutput


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


def update_surge_controller(
    state: UGVState,
    controller_state: SurgeControllerState,
    params: VehicleParams,
    dt: float,
) -> tuple[SurgeControlOutput, SurgeControllerState]:
    """
    Update the super-twisting surge controller.

    Implements:
      - Eq. (11): u_tilde = u - u_d
      - Eq. (16): tau_xc and tau_x1_dot

    Parameters
    ----------
    state : UGVState
        Current vehicle state.
    controller_state : SurgeControllerState
        Internal surge controller state.
    params : VehicleParams
        Controller and vehicle parameters.
    dt : float
        Controller time step [s].

    Returns
    -------
    tuple[SurgeControlOutput, SurgeControllerState]
        The controller output and the updated controller state.
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    # Eq. (11): surge speed tracking error
    u_tilde = state.u - params.desired_speed

    # Eq. (16): continuous super-twisting term before saturation
    raw_tau_xc = (
        -params.k_x * math.sqrt(abs(u_tilde)) * sign(u_tilde)
        + controller_state.tau_x1
    )

    # Eq. (16): saturated machine surge command
    tau_xc = saturate(raw_tau_xc, params.tau_x_max)

    # Eq. (16): internal state derivative
    if abs(tau_xc) >= params.tau_x_max:
        tau_x1_dot = 0.0
    else:
        tau_x1_dot = -params.k_x1 * sign(u_tilde)

    # Discrete-time integration of tau_x1
    new_tau_x1 = controller_state.tau_x1 + tau_x1_dot * dt

    new_state = SurgeControllerState(tau_x1=new_tau_x1)
    output = SurgeControlOutput(
        tau_xc=tau_xc,
        u_tilde=u_tilde,
    )

    return output, new_state