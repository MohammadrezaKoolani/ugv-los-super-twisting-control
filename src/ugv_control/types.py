from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence


@dataclass(frozen=True, slots=True)
class Waypoint:
    """
    A single 2D waypoint in the inertial/NED plane.

    Corresponds to p_k = [x_k, y_k]^T in the paper.
    """
    x: float
    y: float


@dataclass(slots=True)
class Path:
    """
    A piecewise-linear path defined by an ordered list of waypoints.

    A valid path must contain at least two waypoints, so that at least
    one path segment exists.
    """
    waypoints: list[Waypoint] = field(default_factory=list)

    def __post_init__(self) -> None:
        if len(self.waypoints) < 2:
            raise ValueError("Path must contain at least two waypoints.")

    @property
    def num_waypoints(self) -> int:
        return len(self.waypoints)

    @property
    def num_segments(self) -> int:
        return len(self.waypoints) - 1

    def waypoint(self, index: int) -> Waypoint:
        return self.waypoints[index]


@dataclass(frozen=True, slots=True)
class Segment:
    """
    Active path segment from waypoint k to waypoint k+1.
    """
    start: Waypoint
    end: Waypoint
    index: int


@dataclass(frozen=True, slots=True)
class LOSGuidanceOutput:
    """
    Output of the LOS guidance law.

    Variables correspond to the paper as follows:
      - alpha_k : Eq. (21)
      - e       : Eq. (24)
      - e_dot   : Eq. (27)
      - psi_d   : Eq. (33)
      - r_d     : Eq. (41)

    We also keep chi_d optionally for debugging/inspection, even though
    the controller uses psi_d.
    """
    alpha_k: float
    e_ct: float
    e_ct_dot: float
    psi_d: float
    r_d: float
    chi_d: float


@dataclass(frozen=True, slots=True)
class SurgeControlOutput:
    """
    Output of the surge controller.

    tau_xc corresponds to the machine surge command in Eq. (16).
    u_tilde corresponds to the surge speed tracking error in Eq. (11).
    """
    tau_xc: float
    u_tilde: float


@dataclass(frozen=True, slots=True)
class HeadingControlOutput:
    """
    Output of the heading controller.

    tau_psi_c corresponds to the machine heading command in Eq. (44).
    psi_tilde and r_tilde are the tracking errors used by the controller.
    sigma_r is the sliding surface used in Eq. (44).
    """
    tau_psi_c: float
    psi_tilde: float
    r_tilde: float
    sigma_r: float


@dataclass(frozen=True, slots=True)
class ControlOutput:
    """
    Combined machine control output.

    This is the high-level controller result corresponding to Eq. (48):
        tau_c = [tau_xc, 0, tau_psi_c]^T

    Since your project focuses on path following rather than low-level
    actuator physics, we store the two relevant command channels directly.
    """
    tau_xc: float
    tau_psi_c: float