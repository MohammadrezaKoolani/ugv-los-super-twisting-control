from __future__ import annotations

import math

from ugv_control.models.states import UGVState
from ugv_control.types import LOSGuidanceOutput, Segment


def wrap_angle(angle: float) -> float:
    """
    Wrap an angle to [-pi, pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_los_guidance(
    state: UGVState,
    segment: Segment,
    lookahead_distance: float,
) -> LOSGuidanceOutput:
    """
    Compute LOS guidance quantities for the active path segment.

    This implementation follows the paper equations:
      - alpha_k from Eq. (21)
      - e from Eq. (24)
      - e_dot from Eq. (27)
      - chi_d from Eq. (29)
      - psi_d from Eq. (33), using beta = 0
      - r_d from Eq. (41), using beta_dot = 0

    Parameters
    ----------
    state : UGVState
        Current vehicle state.
    segment : Segment
        Active path segment from waypoint k to waypoint k+1.
    lookahead_distance : float
        LOS lookahead distance Delta.

    Returns
    -------
    LOSGuidanceOutput
        Structured LOS guidance result.
    """
    if lookahead_distance <= 0.0:
        raise ValueError("lookahead_distance must be positive.")

    x_k = segment.start.x
    y_k = segment.start.y
    x_k1 = segment.end.x
    y_k1 = segment.end.y

    # Eq. (21): path angle alpha_k
    alpha_k = math.atan2(y_k1 - y_k, x_k1 - x_k)

    # Eq. (24): cross-track error e
    e_ct = (
        -(state.x_n - x_k) * math.sin(alpha_k)
        + (state.y_n - y_k) * math.cos(alpha_k)
    )

    # Eq. (26): inertial velocity components
    x_n_dot = state.u * math.cos(state.psi) - state.v_sway * math.sin(state.psi)
    y_n_dot = state.u * math.sin(state.psi) + state.v_sway * math.cos(state.psi)

    # Eq. (27): cross-track error derivative
    e_ct_dot = -x_n_dot * math.sin(alpha_k) + y_n_dot * math.cos(alpha_k)

    # Eq. (29): desired course angle chi_d
    chi_d = alpha_k + math.atan2(-e_ct, lookahead_distance)
    chi_d = wrap_angle(chi_d)

    # Eq. (33): desired heading psi_d
    # Using beta = 0, as done in the paper experiments
    psi_d = chi_d
    psi_d = wrap_angle(psi_d)

    # Eq. (41): desired yaw rate r_d
    # Using beta_dot = 0, as done in the paper experiments
    r_d = -lookahead_distance / (e_ct * e_ct + lookahead_distance * lookahead_distance) * e_ct_dot

    return LOSGuidanceOutput(
        alpha_k=alpha_k,
        e_ct=e_ct,
        e_ct_dot=e_ct_dot,
        psi_d=psi_d,
        r_d=r_d,
        chi_d=chi_d,
    )