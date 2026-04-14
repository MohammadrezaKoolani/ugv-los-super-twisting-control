from __future__ import annotations

import math

import matplotlib.pyplot as plt

from ugv_control.controllers.manager import step_path_following_controller
from ugv_control.interfaces.eight_wheel_mapper import map_control_to_eight_wheel
from ugv_control.models.states import ControllerState, UGVState, WaypointProgress
from ugv_control.models.vehicle_8ws import build_default_8ws_vehicle
from ugv_control.types import Path, Waypoint
from ugv_control.models.truck_plant_8ws import simulate_step_truck_karl_style

def wrap_angle(angle: float) -> float:
    """
    Wrap angle to [-pi, pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))


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
            Waypoint(x=0.0,   y=0.0),
            Waypoint(x=50.0,  y=0.0),
            Waypoint(x=100.0, y=10.0),
            Waypoint(x=150.0, y=10.0),
            Waypoint(x=200.0, y=0.0),
            Waypoint(x=250.0, y=-10.0),
            Waypoint(x=300.0, y=-10.0),
            Waypoint(x=350.0, y=0.0),
            Waypoint(x=400.0, y=10.0),
            Waypoint(x=450.0, y=10.0),
            Waypoint(x=500.0, y=0.0),
        ]
    )
    
    # path = Path(
    #     waypoints=[
    #         Waypoint(x=0.0,   y=0.0),
    #         Waypoint(x=150.0,  y=0.0),
    #         Waypoint(x=200.0, y=0.0),
    #         Waypoint(x=250.0, y=-0.0),
    #         Waypoint(x=300.0, y=-0.0),
    #         Waypoint(x=350.0, y=0.0),
    #         Waypoint(x=400.0, y=0.0),
    #         Waypoint(x=450.0, y=0.0),
    #         Waypoint(x=500.0, y=0.0),
    #     ]
    # )

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
    max_time = 1000.0
    position_tolerance = 0.5
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
        # if dist_to_goal < 8.0:
        #     params.desired_speed = 1.0
        # if dist_to_goal < 3.0:
        #     params.desired_speed = 0.3

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

        # Drive the truck plant using the mapped 8-wheel high-level command.
        state = simulate_step_truck_karl_style(
            state=state,
            tau_xc=result.control.tau_xc,
            tau_psi_c=result.control.tau_psi_c,
            params=params,
            dt=dt,
            command=truck_cmd,
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

    fig8 = plt.figure()
    ax8 = fig8.add_subplot(111)
    ax8.plot(time_hist, r_hist, label="r")
    ax8.plot(time_hist, r_d_hist, label="r_d")
    ax8.set_xlabel("time [s]")
    ax8.set_ylabel("yaw rate [rad/s]")
    ax8.set_title("Yaw-Rate Tracking")
    ax8.legend()
    ax8.grid(True)

    plt.show()


if __name__ == "__main__":
    main()