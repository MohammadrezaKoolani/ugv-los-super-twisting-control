from __future__ import annotations

import math

from ugv_control.controllers.manager import step_path_following_controller
from ugv_control.guidance.los import compute_los_guidance
from ugv_control.guidance.path import get_segment
from ugv_control.guidance.waypoint_manager import update_waypoint_progress
from ugv_control.models.states import ControllerState, UGVState, WaypointProgress
from ugv_control.models.vehicle_4ws import build_default_4ws_vehicle
from ugv_control.types import Path, Waypoint


def build_test_path() -> Path:
    return Path(
        waypoints=[
            Waypoint(x=0.0, y=0.0),
            Waypoint(x=10.0, y=0.0),
            Waypoint(x=20.0, y=0.0),
        ]
    )


def test_cross_track_error_for_horizontal_path() -> None:
    path = build_test_path()
    segment = get_segment(path, 0)

    state = UGVState(
        x_n=2.0,
        y_n=1.0,
        psi=0.0,
        u=1.0,
        v_sway=0.0,
        r=0.0,
    )

    guidance = compute_los_guidance(
        state=state,
        segment=segment,
        lookahead_distance=1.5,
    )

    assert math.isclose(guidance.e_ct, 1.0, rel_tol=0.0, abs_tol=1e-9)


def test_los_heading_points_back_toward_path() -> None:
    path = build_test_path()
    segment = get_segment(path, 0)

    state = UGVState(
        x_n=2.0,
        y_n=1.0,
        psi=0.0,
        u=1.0,
        v_sway=0.0,
        r=0.0,
    )

    guidance = compute_los_guidance(
        state=state,
        segment=segment,
        lookahead_distance=1.5,
    )

    # Since the vehicle is above the path (positive cross-track error),
    # desired heading should point downward toward the path, so psi_d < 0.
    assert guidance.psi_d < 0.0


def test_surge_command_positive_when_speed_below_desired() -> None:
    path = build_test_path()
    params = build_default_4ws_vehicle()

    state = UGVState(
        x_n=2.0,
        y_n=0.0,
        psi=0.0,
        u=1.5,   # below desired_speed=2.0
        v_sway=0.0,
        r=0.0,
    )

    result = step_path_following_controller(
        state=state,
        params=params,
        controller_state=ControllerState(),
        waypoint_progress=WaypointProgress(segment_index=0),
        path=path,
        dt=0.02,
    )

    assert result.control.tau_xc > 0.0


def test_waypoint_switch_when_inside_acceptance_radius() -> None:
    path = build_test_path()
    progress = WaypointProgress(segment_index=0)

    # Close to waypoint 1 at (10, 0), within radius 2.5
    state = UGVState(
        x_n=9.0,
        y_n=0.0,
        psi=0.0,
        u=0.0,
        v_sway=0.0,
        r=0.0,
    )

    new_progress = update_waypoint_progress(
        state=state,
        path=path,
        progress=progress,
        acceptance_radius=2.5,
    )

    assert new_progress.segment_index == 1