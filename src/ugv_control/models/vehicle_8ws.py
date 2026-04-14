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

        # Guidance: make it much less aggressive for the heavy truck
        lookahead_distance=20.0,
        acceptance_radius=12.0,

        # Surge controller
        k_x=0.075,
        k_x1=0.125,
        tau_x_max=1.0,

        # Heading controller: soften it
        k_r=1.2,
        k_psi=0.02,
        k_psi1=0.03,
        tau_psi_max=1.0,

        # Operating point: slow down for the first stable path-following test
        desired_speed=1.0,

        # 8-wheel geometry
        L1=2.40,
        L2=0.68,
        L3=-0.68,
        L4=-2.40,
        track_width=2.50,

        max_steer_axle1=0.45,
        max_steer_axle2=0.35,

        max_drive_torque_per_wheel=1500.0,
        num_driven_wheels=4,

        # High-level lumped plant parameters
        wheel_radius=0.58,
        yaw_time_constant=0.9,
        sway_time_constant=1.2,
        beta_gain=0.75,
    )