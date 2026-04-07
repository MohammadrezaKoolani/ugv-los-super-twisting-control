from __future__ import annotations

from ugv_control.models.vehicle_base import VehicleParams


def build_default_4ws_vehicle() -> VehicleParams:
    """
    Create a default parameter set for a 4-wheel steering vehicle.

    The values below are placeholders / initial defaults inspired by the
    structure of the paper. They should be tuned for your real platform.

    Returns
    -------
    VehicleParams
        Parameter set used by guidance and control modules.
    """
    return VehicleParams(
        # Dynamic/model parameters
        m=13.8,
        I_z=1.12,
        X_u_abs_u=-1.0,

        # Guidance parameters
        lookahead_distance=4.0,
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