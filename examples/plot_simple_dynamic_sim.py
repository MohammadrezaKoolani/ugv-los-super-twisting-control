from __future__ import annotations

import math

import matplotlib.pyplot as plt

from ugv_control.controllers.manager import step_path_following_controller
from ugv_control.interfaces.eight_wheel_mapper import map_control_to_eight_wheel
from ugv_control.models.states import ControllerState, UGVState, WaypointProgress
from ugv_control.models.vehicle_8ws import build_default_8ws_vehicle
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
    Simple closed-loop vehicle simulation used only for controller testing.

    This is still a command-level plant, not the real heavy-truck plant:
      - tau_xc is treated as a normalized throttle-like command
      - tau_psi_c is treated as a desired yaw-rate command
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    a_u = 1.5
    a_r = 12.0

    # Interpret tau_xc as normalized command in [0, 1]
    u_max_sim = 8.0
    tau_xc_clamped = max(0.0, min(1.0, tau_xc))
    u_ref = tau_xc_clamped * u_max_sim

    # Interpret tau_psi_c as normalized steering/yaw command in [-1, 1]
    r_max_sim = 1.2
    tau_psi_c_clamped = max(-1.0, min(1.0, tau_psi_c))
    r_ref = tau_psi_c_clamped * r_max_sim

    u_dot = a_u * (u_ref - state.u)
    r_dot = a_r * (r_ref - state.r)

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



def has_passed_final_waypoint_along_last_segment(
    state: UGVState,
    path: Path,
) -> bool:
    """
    Return True if the vehicle has passed the final waypoint when projected
    onto the last path segment direction.
    """
    if len(path.waypoints) < 2:
        return False

    wp_prev = path.waypoints[-2]
    wp_final = path.waypoints[-1]

    seg_dx = wp_final.x - wp_prev.x
    seg_dy = wp_final.y - wp_prev.y
    seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy

    if seg_len_sq <= 1e-12:
        return False

    # projection of vehicle position onto last segment direction
    veh_dx = state.x_n - wp_prev.x
    veh_dy = state.y_n - wp_prev.y
    proj = (veh_dx * seg_dx + veh_dy * seg_dy) / seg_len_sq

    # proj > 1 means vehicle is past the final waypoint along segment direction
    return proj >= 1.0

def should_stop_simulation(
    state: UGVState,
    path: Path,
    position_tolerance: float,
) -> bool:
    """
    Stop if the vehicle is near the final waypoint, or if it has already
    passed the final waypoint along the final segment direction.
    """
    final_wp = path.waypoints[-1]
    dist = math.hypot(state.x_n - final_wp.x, state.y_n - final_wp.y)

    if dist <= position_tolerance:
        return True

    if has_passed_final_waypoint_along_last_segment(state, path):
        return True

    return False

def main() -> None:
    path = Path(
        waypoints=[
            Waypoint(x=0.0, y=0.0),
            Waypoint(x=15.0, y=0.0),
            Waypoint(x=30.0, y=0.0),
            Waypoint(x=45.0, y=5.0),
        ]
    )

    # Use the heavy 8-wheel truck parameters
    params = build_default_8ws_vehicle()

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

    dt = 0.01
    max_time = 60.0
    position_tolerance = 0.5
    speed_tolerance = 0.2
    time_now = 0.0

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
    desired_speed_hist: list[float] = []

    # 8-wheel mapped actuator histories
    steer_axle1_hist: list[float] = []
    steer_axle2_hist: list[float] = []
    tq_3l_hist: list[float] = []
    tq_3r_hist: list[float] = []
    tq_4l_hist: list[float] = []
    tq_4r_hist: list[float] = []

    #tore the original desired speed once
    base_desired_speed = params.desired_speed

    while time_now <= max_time:

        final_wp = path.waypoints[-1]
        dist_to_goal = math.hypot(state.x_n - final_wp.x, state.y_n - final_wp.y)

        params.desired_speed = base_desired_speed
        if dist_to_goal < 8.0:
            params.desired_speed = 1.0
        if dist_to_goal < 3.0:
            params.desired_speed = 0.3

        result = step_path_following_controller(
            state=state,
            params=params,
            controller_state=controller_state,
            waypoint_progress=waypoint_progress,
            path=path,
            dt=dt,
        )

        # This is the new step:
        # convert high-level Karl-style controller outputs into real
        # 8-wheel truck actuator commands.
        truck_cmd = map_control_to_eight_wheel(
            control=result.control,
            params=params,
        )

        time_hist.append(time_now)
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
        desired_speed_hist.append(params.desired_speed)

        steer_axle1_hist.append(truck_cmd.steer_axle1)
        steer_axle2_hist.append(truck_cmd.steer_axle2)
        tq_3l_hist.append(truck_cmd.torque_axle3_left)
        tq_3r_hist.append(truck_cmd.torque_axle3_right)
        tq_4l_hist.append(truck_cmd.torque_axle4_left)
        tq_4r_hist.append(truck_cmd.torque_axle4_right)

        # For now, the simulator still uses the high-level controller outputs.
        # That is okay for development. Later, the real truck will receive truck_cmd.
        state = simulate_step_dynamic(
            state=state,
            tau_xc=result.control.tau_xc,
            tau_psi_c=result.control.tau_psi_c,
            dt=dt,
        )

        controller_state = result.controller_state
        waypoint_progress = result.waypoint_progress

        time_now += dt

        if should_stop_simulation(
            state=state,
            path=path,
            position_tolerance=position_tolerance,
        ):
            break



    #===========
    # Plotting
    #===========

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
    ax3.plot(time_hist, desired_speed_hist, linestyle="--", label="desired speed")
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
    ax4.set_title("High-Level Controller Outputs")
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

    fig6 = plt.figure()
    ax6 = fig6.add_subplot(111)
    ax6.plot(time_hist, steer_axle1_hist, label="steer axle 1")
    ax6.plot(time_hist, steer_axle2_hist, label="steer axle 2")
    ax6.set_xlabel("time [s]")
    ax6.set_ylabel("steering angle [rad]")
    ax6.set_title("Mapped 8-Wheel Steering Commands")
    ax6.legend()
    ax6.grid(True)

    fig7 = plt.figure()
    ax7 = fig7.add_subplot(111)
    ax7.plot(time_hist, tq_3l_hist, label="axle3 left")
    ax7.plot(time_hist, tq_3r_hist, label="axle3 right")
    ax7.plot(time_hist, tq_4l_hist, label="axle4 left")
    ax7.plot(time_hist, tq_4r_hist, label="axle4 right")
    ax7.set_xlabel("time [s]")
    ax7.set_ylabel("wheel torque [N m]")
    ax7.set_title("Mapped 8-Wheel Drive Torques")
    ax7.legend()
    ax7.grid(True)

    plt.show()


if __name__ == "__main__":
    main()