from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class VehicleParams:
    """
    Common vehicle, guidance, and controller parameters.

    This class contains the parameters needed by the LOS guidance law
    and the super-twisting surge and heading controllers.

    Model-related parameters:
      - m        : mass, from Eq. (6)
      - I_z      : yaw inertia, from Eq. (6)
      - X_u_abs_u: quadratic surge drag coefficient, from Eq. (8)

    Guidance parameters:
      - lookahead_distance  : Delta, from Eqs. (29), (33), (41)
      - acceptance_radius   : R_k, from Eq. (10)

    Surge controller parameters:
      - k_x       : Eq. (16)
      - k_x1      : Eq. (16)
      - tau_x_max : Eq. (16)

    Heading controller parameters:
      - k_r         : sliding surface gain, from Eq. (44)
      - k_psi       : Eq. (44)
      - k_psi1      : Eq. (44)
      - tau_psi_max : Eq. (44)

    Optional operating parameters:
      - desired_speed : u_d from Eq. (11)
    """
    # Dynamic/model parameters
    # Karl-style dynamic/model parameters
    m: float
    I_z: float
    X_u_abs_u: float

    # LOS guidance
    lookahead_distance: float
    acceptance_radius: float

    # Surge controller
    k_x: float
    k_x1: float
    tau_x_max: float

    # Heading controller
    k_r: float
    k_psi: float
    k_psi1: float
    tau_psi_max: float

    # Desired speed
    desired_speed: float = 0.0

    # 8-wheel truck geometry / mapping parameters
    L1: float = 0.0
    L2: float = 0.0
    L3: float = 0.0
    L4: float = 0.0
    track_width: float = 0.0

    max_steer_axle1: float = 0.0
    max_steer_axle2: float = 0.0

    max_drive_torque_per_wheel: float = 0.0
    num_driven_wheels: int = 4

    def __post_init__(self) -> None:
        if self.m <= 0.0:
            raise ValueError("m must be positive.")
        if self.I_z <= 0.0:
            raise ValueError("I_z must be positive.")
        if self.lookahead_distance <= 0.0:
            raise ValueError("lookahead_distance must be positive.")
        if self.acceptance_radius <= 0.0:
            raise ValueError("acceptance_radius must be positive.")
        if self.k_x <= 0.0:
            raise ValueError("k_x must be positive.")
        if self.k_x1 <= 0.0:
            raise ValueError("k_x1 must be positive.")
        if self.tau_x_max <= 0.0:
            raise ValueError("tau_x_max must be positive.")
        if self.k_r <= 0.0:
            raise ValueError("k_r must be positive.")
        if self.k_psi <= 0.0:
            raise ValueError("k_psi must be positive.")
        if self.k_psi1 <= 0.0:
            raise ValueError("k_psi1 must be positive.")
        if self.tau_psi_max <= 0.0:
            raise ValueError("tau_psi_max must be positive.")
        
        