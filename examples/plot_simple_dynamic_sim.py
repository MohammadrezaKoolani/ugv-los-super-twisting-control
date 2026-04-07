from __future__ import annotations

import math

import matplotlib.pyplot as plt

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

    a_u = 1.5
    a_r = 2.5

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

    time_hist: list[float] = []
    x_hist: list[float] = []
    y_hist: list[float] = []
    psi_hist: list[float] = []
    u_hist: list[float] = []
    r_hist: list[float] = []
    e_hist: list[float] = []
    psi_d_hist: list[float] = []
    r_d_hist: list[float] = []
    tau_x_hist: list[float] = []
    tau_psi_hist: list[float] = []
    seg_hist: list[int] = []

    for step in range(steps):
        result = step_path_following_controller(
            state=state,
            params=params,
            controller_state=controller_state,
            waypoint_progress=waypoint_progress,
            path=path,
            dt=dt,
        )

        time_hist.append(step * dt)
        x_hist.append(state.x_n)
        y_hist.append(state.y_n)
        psi_hist.append(state.psi)
        u_hist.append(state.u)
        r_hist.append(state.r)
        e_hist.append(result.guidance.e_ct)
        psi_d_hist.append(result.guidance.psi_d)
        r_d_hist.append(result.guidance.r_d)
        tau_x_hist.append(result.control.tau_xc)
        tau_psi_hist.append(result.control.tau_psi_c)
        seg_hist.append(result.waypoint_progress.segment_index)

        state = simulate_step_dynamic(
            state=state,
            tau_xc=result.control.tau_xc,
            tau_psi_c=result.control.tau_psi_c,
            dt=dt,
        )

        controller_state = result.controller_state
        waypoint_progress = result.waypoint_progress

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(x_hist, y_hist, label="vehicle trajectory")
    ax1.plot(
        [wp.x for wp in path.waypoints],
        [wp.y for wp in path.waypoints],
        "o--",
        label="path waypoints",
    )
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_title("Trajectory vs Path")
    ax1.legend()
    ax1.grid(True)

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(time_hist, e_hist)
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("cross-track error e_ct [m]")
    ax2.set_title("Cross-Track Error")
    ax2.grid(True)

    fig3 = plt.figure()
    ax3 = fig3.add_subplot(111)
    ax3.plot(time_hist, u_hist, label="u")
    ax3.axhline(params.desired_speed, linestyle="--", label="desired speed")
    ax3.set_xlabel("time [s]")
    ax3.set_ylabel("surge speed [m/s]")
    ax3.set_title("Speed Response")
    ax3.legend()
    ax3.grid(True)

    fig4 = plt.figure()
    ax4 = fig4.add_subplot(111)
    ax4.plot(time_hist, tau_x_hist, label="tau_xc")
    ax4.plot(time_hist, tau_psi_hist, label="tau_psi_c")
    ax4.set_xlabel("time [s]")
    ax4.set_ylabel("control command")
    ax4.set_title("Controller Outputs")
    ax4.legend()
    ax4.grid(True)

    fig5 = plt.figure()
    ax5 = fig5.add_subplot(111)
    ax5.plot(time_hist, psi_hist, label="psi")
    ax5.plot(time_hist, psi_d_hist, label="psi_d")
    ax5.set_xlabel("time [s]")
    ax5.set_ylabel("heading [rad]")
    ax5.set_title("Heading Tracking")
    ax5.legend()
    ax5.grid(True)

    plt.show()


if __name__ == "__main__":
    main()