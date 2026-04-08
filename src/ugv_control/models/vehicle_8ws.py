from __future__ import annotations

from ugv_control.models.vehicle_base import VehicleParams


def build_default_8ws_vehicle() -> VehicleParams:
    """
    Heavy 8-wheel dump truck parameters for Karl-style LOS + super-twisting control.
    """
    return VehicleParams(
        # Karl-style dynamic parameters
        m=25000.0,
        I_z=98000.0,
        X_u_abs_u=-150.0,   # initial guess, tune later

        # LOS guidance
        lookahead_distance=6.0,
        acceptance_radius=4.0,

        # Surge controller
        k_x=0.075,
        k_x1=0.125,
        tau_x_max=1.0,      # normalized drive command

        # Heading controller
        k_r=1.0,
        k_psi=0.1,
        k_psi1=0.2,
        tau_psi_max=1.0,    # normalized steering/yaw command

        # Desired speed
        desired_speed=5.0,
    )


