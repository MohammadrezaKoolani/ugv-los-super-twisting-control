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
        X_u_abs_u=-150.0,

        lookahead_distance=6.0,
        acceptance_radius=4.0,

        k_x=0.075,
        k_x1=0.125,
        tau_x_max=10.0,

        k_r=1.0,
        k_psi=0.1,
        k_psi1=0.2,
        tau_psi_max=1.0,

        desired_speed=5.0,

        L1=2.40,
        L2=0.68,
        L3=-0.68,
        L4=-2.40,
        track_width=2.50,

        max_steer_axle1=0.45,
        max_steer_axle2=0.35,

        max_drive_torque_per_wheel=8000.0,
        num_driven_wheels=4,
)