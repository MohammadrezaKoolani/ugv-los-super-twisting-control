from __future__ import annotations

from dataclasses import dataclass

from ugv_control.controllers.heading import update_heading_controller
from ugv_control.controllers.surge import update_surge_controller
from ugv_control.guidance.los import compute_los_guidance
from ugv_control.guidance.path import get_segment
from ugv_control.guidance.waypoint_manager import update_waypoint_progress
from ugv_control.models.states import ControllerState, UGVState, WaypointProgress
from ugv_control.models.vehicle_base import VehicleParams
from ugv_control.types import ControlOutput, LOSGuidanceOutput


@dataclass(frozen=True, slots=True)
class ControllerStepResult:
    """
    Full result of one controller update step.

    This is useful because the caller may want:
      - the combined control output
      - the updated controller internal state
      - the updated waypoint/segment progress
      - the LOS guidance values for logging/debugging
    """
    control: ControlOutput
    controller_state: ControllerState
    waypoint_progress: WaypointProgress
    guidance: LOSGuidanceOutput


def step_path_following_controller(
    state: UGVState,
    params: VehicleParams,
    controller_state: ControllerState,
    waypoint_progress: WaypointProgress,
    path,
    dt: float,
) -> ControllerStepResult:
    """
    Execute one full path-following controller step.

    Pipeline:
      1) update active waypoint/segment using Eq. (10)
      2) get active segment
      3) compute LOS guidance
      4) update surge controller
      5) update heading controller
      6) combine outputs into Eq. (48)

    Parameters
    ----------
    state : UGVState
        Current vehicle state.
    params : VehicleParams
        Vehicle/controller parameter set.
    controller_state : ControllerState
        Current internal controller states.
    waypoint_progress : WaypointProgress
        Current active segment index.
    path : Path
        Piecewise-linear waypoint path.
    dt : float
        Controller time step [s].

    Returns
    -------
    ControllerStepResult
        Full result of one controller step.
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    # Step 1: update active segment index using waypoint acceptance logic (Eq. 10)
    new_progress = update_waypoint_progress(
        state=state,
        path=path,
        progress=waypoint_progress,
        acceptance_radius=params.acceptance_radius,
    )

    # Step 2: extract active segment
    segment = get_segment(path, new_progress.segment_index)

    # Step 3: LOS guidance
    guidance = compute_los_guidance(
        state=state,
        segment=segment,
        lookahead_distance=params.lookahead_distance,
    )

    # Step 4: surge controller
    surge_output, new_surge_state = update_surge_controller(
        state=state,
        controller_state=controller_state.surge,
        params=params,
        dt=dt,
    )

    # Step 5: heading controller
    heading_output, new_heading_state = update_heading_controller(
        state=state,
        guidance=guidance,
        controller_state=controller_state.heading,
        params=params,
        dt=dt,
    )

    # Step 6: combined machine control output (Eq. 48)
    control = ControlOutput(
        tau_xc=surge_output.tau_xc,
        tau_psi_c=heading_output.tau_psi_c,
    )

    new_controller_state = ControllerState(
        surge=new_surge_state,
        heading=new_heading_state,
    )

    return ControllerStepResult(
        control=control,
        controller_state=new_controller_state,
        waypoint_progress=new_progress,
        guidance=guidance,
    )