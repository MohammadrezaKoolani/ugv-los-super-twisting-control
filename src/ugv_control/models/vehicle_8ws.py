from __future__ import annotations

from ugv_control.models.vehicle_base import VehicleParams


def build_default_8ws_vehicle() -> VehicleParams:
    """
    Create a default parameter set for an 8-wheel vehicle.

    These values are initial placeholders and should be replaced or tuned
    for the real 8-wheel platform.

    Returns
    -------
    VehicleParams
        Parameter set used by guidance and control modules.
    """
    return VehicleParams(
        # Dynamic/model parameters
        m=25.0,
        I_z=2.5,
        X_u_abs_u=-1.0,

        # Guidance parameters
        lookahead_distance=1.5,
        acceptance_radius=2.5,

        # Surge controller gains/limits
        k_x=0.075,
        k_x1=0.125,
        tau_x_max=100.0,

        # Heading controller gains/limits
        k_r=1.0,
        k_psi=0.1,
        k_psi1=0.2,
        tau_psi_max=20.0,

        # Desired forward speed
        desired_speed=2.0,
    )