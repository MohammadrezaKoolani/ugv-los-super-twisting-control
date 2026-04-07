from __future__ import annotations

import math

from ugv_control.controllers.manager import step_path_following_controller
from ugv_control.models.states import ControllerState, UGVState, WaypointProgress
from ugv_control.models.vehicle_4ws import build_default_4ws_vehicle
from ugv_control.types import Path, Waypoint


def simulate_step(
    state: UGVState,
    tau_xc: float,
    tau_psi_c: float,
    dt: float,
) -> UGVState:
    """
    Very simple kinematic-like propagation.

    This is not the full dynamic model from the paper.
    We use a lightweight approximation:

      u_dot ~= tau_xc
      r     ~= tau_psi_c
      v_sway = 0

    Then propagate:
      x_dot = u cos(psi)
      y_dot = u sin(psi)
      psi_dot = r

    The purpose is only to verify that the controller structure can
    drive the vehicle toward the path in a simple simulation.
    """
    # Approximate longitudinal speed update
    new_u = state.u + tau_xc * dt

    # Approximate yaw-rate assignment
    new_r = tau_psi_c

    # Keep sway speed zero in this simple simulator
    new_v_sway = 0.0

    # Kinematics
    x_dot = new_u * math.cos(state.psi)
    y_dot = new_u * math.sin(state.psi)
    psi_dot = new_r

    new_x = state.x_n + x_dot * dt
    new_y = state.y_n + y_dot * dt
    new_psi = state.psi + psi_dot * dt

    return UGVState(
        x_n=new_x,
        y_n=new_y,
        psi=new_psi,
        u=new_u,
        v_sway=new_v_sway,
        r=new_r,
    )


def main() -> None:
    path = Path(
        waypoints=[
            Waypoint(x=0.0, y=0.0),
            Waypoint(x=15.0, y=0.0),
            Waypoint(x=30.0, y=0.0),
            Waypoint(x=45.0, y=5.0),
        ]
    )

    params = build_default_4ws_vehicle()

    state = UGVState(
        x_n=0.0,
        y_n=3.0,
        psi=0.2,
        u=0.0,
        v_sway=0.0,
        r=0.0,
    )

    controller_state = ControllerState()
    waypoint_progress = WaypointProgress(segment_index=0)

    dt = 0.02
    steps = 500

    print("Starting simple closed-loop simulation...")
    print("-" * 60)

    for step in range(steps):
        result = step_path_following_controller(
            state=state,
            params=params,
            controller_state=controller_state,
            waypoint_progress=waypoint_progress,
            path=path,
            dt=dt,
        )

        state = simulate_step(
            state=state,
            tau_xc=result.control.tau_xc,
            tau_psi_c=result.control.tau_psi_c,
            dt=dt,
        )

        controller_state = result.controller_state
        waypoint_progress = result.waypoint_progress

        if step % 25 == 0 or step == steps - 1:
            print(
                f"step={step:04d} "
                f"x={state.x_n:7.3f} "
                f"y={state.y_n:7.3f} "
                f"psi={state.psi:7.3f} "
                f"u={state.u:7.3f} "
                f"seg={waypoint_progress.segment_index} "
                f"e_ct={result.guidance.e_ct:7.3f} "
                f"psi_d={result.guidance.psi_d:7.3f} "
                f"tau_xc={result.control.tau_xc:7.3f} "
                f"tau_psi_c={result.control.tau_psi_c:7.3f}"
            )

    print("-" * 60)
    print("Simulation finished.")


if __name__ == "__main__":
    main()