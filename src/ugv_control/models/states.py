from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class UGVState:
    """
    Full vehicle state used by guidance and control.

    Coordinates and variables follow the paper notation:
      - x_n, y_n, psi come from eta in Eq. (4)
      - u, v, r come from the body-fixed velocity vector in Eq. (4)

    In code, we rename the sway velocity from `v` to `v_sway`
    to avoid confusion with the velocity vector notation.
    """
    x_n: float = 0.0
    y_n: float = 0.0
    psi: float = 0.0
    u: float = 0.0
    v_sway: float = 0.0
    r: float = 0.0


@dataclass(slots=True)
class WaypointProgress:
    """
    Keeps track of which path segment is currently active.

    If the path is defined by waypoints p_0, p_1, ..., p_N,
    then `segment_index = k` means the active segment is from
    waypoint k to waypoint k+1.
    """
    segment_index: int = 0


@dataclass(slots=True)
class SurgeControllerState:
    """
    Internal state of the super-twisting surge controller.

    tau_x1 corresponds to the internal integrator state in Eq. (16).
    """
    tau_x1: float = 0.0


@dataclass(slots=True)
class HeadingControllerState:
    """
    Internal state of the super-twisting heading controller.

    tau_psi1 corresponds to the internal integrator state in Eq. (44).
    """
    tau_psi1: float = 0.0


@dataclass(slots=True)
class ControllerState:
    """
    Groups all controller internal states together.
    """
    surge: SurgeControllerState = field(default_factory=SurgeControllerState)
    heading: HeadingControllerState = field(default_factory=HeadingControllerState)