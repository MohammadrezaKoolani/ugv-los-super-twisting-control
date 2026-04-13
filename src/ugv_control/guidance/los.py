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

    High-level simplified LOS form:
      - alpha_k from Eq. (21)
      - e from Eq. (24)
      - e_dot from the simplified UGV-style form
      - psi_d = alpha_k + atan2(-e, Delta)
      - r_d   = -(Delta / (e^2 + Delta^2)) * e_dot

    Notes
    -----
    For the current high-level truck implementation, we intentionally keep:
      - beta = 0
      - beta_dot = 0

    So there is no tire-stiffness-based slip compensation here.
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

    # Simplified UGV-style cross-track error derivative
    # Equivalent to the inertial-velocity expression, but kept in the
    # compact form we want for the current high-level setup.
    e_ct_dot = (
        -state.u * math.sin(alpha_k - state.psi)
        + state.v_sway * math.cos(alpha_k - state.psi)
    )

    # Desired course angle
    chi_d = wrap_angle(alpha_k + math.atan2(-e_ct, lookahead_distance))

    # Desired heading: beta = 0
    psi_d = chi_d

    # Desired yaw rate: beta_dot = 0
    denom = e_ct * e_ct + lookahead_distance * lookahead_distance
    r_d = -(lookahead_distance / denom) * e_ct_dot

    return LOSGuidanceOutput(
        alpha_k=alpha_k,
        e_ct=e_ct,
        e_ct_dot=e_ct_dot,
        psi_d=psi_d,
        r_d=r_d,
        chi_d=chi_d,
    )