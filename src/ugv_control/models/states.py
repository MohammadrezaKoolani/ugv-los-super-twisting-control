from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class VehicleState:
    # Inertial/world-frame pose
    x_n: float
    y_n: float
    psi: float

    # Body-frame velocities
    u: float
    v: float
    r: float


@dataclass(slots=True)
class VehicleParams:
    m: float
    I_z: float
    X_u_abs_u: float  # corresponds to X_{u|u|}


@dataclass(slots=True)
class DisturbanceEstimate:
    d_x: float = 0.0
    d_psi: float = 0.0


@dataclass(slots=True)
class GuidanceConfig:
    lookahead_distance: float


@dataclass(slots=True)
class SurgeControllerConfig:
    k_x: float
    k_x1: float
    tau_x_max: float


@dataclass(slots=True)
class HeadingControllerConfig:
    k_r: float
    k_psi: float
    k_psi1: float
    tau_psi_max: float