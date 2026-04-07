from __future__ import annotations

import math

from ugv_control.controllers.manager import step_path_following_controller
from ugv_control.models.states import ControllerState, UGVState, WaypointProgress
from ugv_control.models.vehicle_4ws import build_default_4ws_vehicle
from ugv_control.types import Path, Waypoint


def wrap_angle(angle: float) -> float:
    """
    Wrap angle to [-pi, pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def simulate_step_dynamic(
    state: UGVState,
    tau_xc: float,
    tau_psi_c: float,
    dt: float,
) -> UGVState:
    """
    Command-level vehicle simulation.

    This is not a force/moment plant. Instead:
      - tau_xc is treated as a desired longitudinal command
      - tau_psi_c is treated as a desired yaw-rate command

    The vehicle responds with first-order dynamics:
      u_dot = a_u * (tau_xc - u)
      r_dot = a_r * (tau_psi_c - r)

    Then kinematics are propagated by:
      x_dot = u cos(psi)
      y_dot = u sin(psi)
      psi_dot = r
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    # Tunable response rates
    a_u = 1.5
    a_r = 2.5

    # First-order response in speed and yaw rate
    u_dot = a_u * (tau_xc - state.u)
    r_dot = a_r * (tau_psi_c - state.r)

    new_u = state.u + u_dot * dt
    new_r = state.r + r_dot * dt
    new_v_sway = 0.0

    x_dot = new_u * math.cos(state.psi)
    y_dot = new_u * math.sin(state.psi)
    psi_dot = new_r

    new_x = state.x_n + x_dot * dt
    new_y = state.y_n + y_dot * dt
    new_psi = wrap_angle(state.psi + psi_dot * dt)

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
    steps = 1000

    print("Starting simplified dynamic closed-loop simulation...")
    print("-" * 80)

    for step in range(steps):
        result = step_path_following_controller(
            state=state,
            params=params,
            controller_state=controller_state,
            waypoint_progress=waypoint_progress,
            path=path,
            dt=dt,
        )

        state = simulate_step_dynamic(
        state=state,
        tau_xc=result.control.tau_xc,
        tau_psi_c=result.control.tau_psi_c,
        dt=dt,
        )

        controller_state = result.controller_state
        waypoint_progress = result.waypoint_progress

        if step % 50 == 0 or step == steps - 1:
            print(
                f"step={step:04d} "
                f"x={state.x_n:8.3f} "
                f"y={state.y_n:8.3f} "
                f"psi={state.psi:8.3f} "
                f"u={state.u:8.3f} "
                f"r={state.r:8.3f} "
                f"seg={waypoint_progress.segment_index} "
                f"e_ct={result.guidance.e_ct:8.3f} "
                f"psi_d={result.guidance.psi_d:8.3f} "
                f"tau_xc={result.control.tau_xc:8.3f} "
                f"tau_psi_c={result.control.tau_psi_c:8.3f}"
            )

    print("-" * 80)
    print("Simulation finished.")


if __name__ == "__main__":
    main()