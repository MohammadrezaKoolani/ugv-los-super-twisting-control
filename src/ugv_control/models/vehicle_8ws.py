from __future__ import annotations

from ugv_control.models.vehicle_base import VehicleParams


def build_default_8ws_vehicle() -> VehicleParams:
    """
    Heavy 8-wheel truck parameters for low-speed LOS + super-twisting control.
    Tuned for the current tire-stiffness-free simulation plant.
    """
    return VehicleParams(
        # Lumped vehicle parameters
        m=25000.0,
        I_z=98000.0,
        X_u_abs_u=-150.0,

        # Guidance
        lookahead_distance=20.0,
        acceptance_radius=10.0,

        # Surge controller
        k_x=0.075,
        k_x1=0.125,
        tau_x_max=1.0,

        # Heading controller
        k_r=1.0,
        k_psi=0.1,
        k_psi1=0.2,
        tau_psi_max=1.0,

        # Operating point
        desired_speed=1.0,

        # 8-wheel geometry
        L1=2.40,
        L2=0.68,
        L3=-0.68,
        L4=-2.40,
        track_width=2.50,

        max_steer_axle1=0.25,
        max_steer_axle2=0.16,

        # Keep these at your real-truck values when known
        max_drive_torque_per_wheel=1500.0,
        num_driven_wheels=4,

        # Lumped plant parameters
        wheel_radius=0.8,
        yaw_time_constant=1.8,
        sway_time_constant=2.5,
        beta_gain=0.4,
    )