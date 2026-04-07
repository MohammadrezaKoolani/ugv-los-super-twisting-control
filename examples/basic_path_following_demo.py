from __future__ import annotations

from ugv_control.controllers.manager import step_path_following_controller
from ugv_control.models.states import ControllerState, UGVState, WaypointProgress
from ugv_control.models.vehicle_4ws import build_default_4ws_vehicle
from ugv_control.types import Path, Waypoint


def main() -> None:
    # Build a simple straight-line path
    path = Path(
        waypoints=[
            Waypoint(x=0.0, y=0.0),
            Waypoint(x=10.0, y=0.0),
            Waypoint(x=20.0, y=0.0),
        ]
    )

    # Example vehicle parameter set
    params = build_default_4ws_vehicle()

    # Example current vehicle state
    # Vehicle is slightly above the path, heading roughly forward,
    # moving a bit slower than desired.
    state = UGVState(
        x_n=2.0,
        y_n=1.0,
        psi=0.05,
        u=1.5,
        v_sway=0.0,
        r=0.0,
    )

    # Initial controller and waypoint states
    controller_state = ControllerState()
    waypoint_progress = WaypointProgress(segment_index=0)

    # One controller step
    dt = 0.02
    result = step_path_following_controller(
        state=state,
        params=params,
        controller_state=controller_state,
        waypoint_progress=waypoint_progress,
        path=path,
        dt=dt,
    )

    print("=== Controller step result ===")
    print(f"Active segment index: {result.waypoint_progress.segment_index}")
    print(f"Cross-track error e_ct: {result.guidance.e_ct:.6f}")
    print(f"Cross-track error rate e_ct_dot: {result.guidance.e_ct_dot:.6f}")
    print(f"Desired heading psi_d [rad]: {result.guidance.psi_d:.6f}")
    print(f"Desired yaw rate r_d [rad/s]: {result.guidance.r_d:.6f}")
    print(f"Surge command tau_xc: {result.control.tau_xc:.6f}")
    print(f"Heading command tau_psi_c: {result.control.tau_psi_c:.6f}")


if __name__ == "__main__":
    main()