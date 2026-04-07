from __future__ import annotations

from ugv_control.models.states import UGVState, WaypointProgress
from ugv_control.types import Path


def update_waypoint_progress(
    state: UGVState,
    path: Path,
    progress: WaypointProgress,
    acceptance_radius: float,
) -> WaypointProgress:
    """
    Update the active path segment index using the waypoint acceptance rule.

    The switching condition follows Eq. (10) of the paper:
        (x_n - x_{k+1})^2 + (y_n - y_{k+1})^2 <= R_{k+1}^2

    Parameters
    ----------
    state : UGVState
        Current vehicle state.
    path : Path
        Piecewise-linear path.
    progress : WaypointProgress
        Current path progress state.
    acceptance_radius : float
        Acceptance circle radius.

    Returns
    -------
    WaypointProgress
        Updated waypoint progress.
    """
    if acceptance_radius <= 0.0:
        raise ValueError("acceptance_radius must be positive.")

    k = progress.segment_index

    # If already on the last segment, do not advance further.
    if k >= path.num_segments - 1:
        return progress

    next_waypoint = path.waypoint(k + 1)

    dx = state.x_n - next_waypoint.x
    dy = state.y_n - next_waypoint.y

    inside_acceptance_circle = (dx * dx + dy * dy) <= (acceptance_radius * acceptance_radius)

    if inside_acceptance_circle:
        return WaypointProgress(segment_index=k + 1)

    return progress