from __future__ import annotations

import math

from ugv_control.controllers.heading import HeadingSuperTwistingController
from ugv_control.controllers.surge import SurgeSpeedController
from ugv_control.guidance.los import LOSGuidance
from ugv_control.models.states import (
    DisturbanceEstimate,
    HeadingControllerConfig,
    SurgeControllerConfig,
    VehicleParams,
    VehicleState,
)
from ugv_control.types import PathSegment


def main() -> None:
    params = VehicleParams(
        m=13.8,
        I_z=1.12,
        X_u_abs_u=-0.5,
    )

    state = VehicleState(
        x_n=0.0,
        y_n=2.0,
        psi=0.2,
        u=1.0,
        v=0.0,
        r=0.0,
    )

    segment = PathSegment(
        x_k=0.0,
        y_k=0.0,
        x_k1=20.0,
        y_k1=0.0,
    )

    los = LOSGuidance(lookahead_distance=1.5)

    surge_controller = SurgeSpeedController(
        SurgeControllerConfig(
            k_x=0.075,
            k_x1=0.125,
            tau_x_max=100.0,
        )
    )

    heading_controller = HeadingSuperTwistingController(
        HeadingControllerConfig(
            k_r=1.0,
            k_psi=0.1,
            k_psi1=0.2,
            tau_psi_max=20.0,
        )
    )

    disturbance = DisturbanceEstimate(d_x=0.0, d_psi=0.0)

    dt = 0.05
    u_d = 2.0

    for step in range(20):
        los_out = los.compute(state, segment)

        surge_out = surge_controller.compute(
            state=state,
            params=params,
            u_d=u_d,
            u_d_dot=0.0,
            disturbance=disturbance,
            dt=dt,
        )

        heading_out = heading_controller.compute(
            state=state,
            params=params,
            psi_d=los_out.psi_d,
            r_d=los_out.r_d,
            r_d_dot=0.0,
            disturbance=disturbance,
            dt=dt,
        )

        print(f"step={step}")
        print(f"  cross_track_error e = {los_out.e:.3f}")
        print(f"  desired_heading psi_d = {los_out.psi_d:.3f}")
        print(f"  desired_yaw_rate r_d = {los_out.r_d:.3f}")
        print(f"  tau_x_c = {surge_out.tau_x_c:.3f}")
        print(f"  tau_psi_c = {heading_out.tau_psi_c:.3f}")
        print()

        # Dummy state propagation just to see values evolve a bit.
        # Replace this later with your real vehicle model integration.
        state.u += 0.05 * (u_d - state.u)
        state.r += 0.1 * (los_out.r_d - state.r)
        state.psi += state.r * dt
        state.x_n += state.u * math.cos(state.psi) * dt
        state.y_n += state.u * math.sin(state.psi) * dt


if __name__ == "__main__":
    main()