from __future__ import annotations

import math

from ugv_control.models.states import VehicleState
from ugv_control.types import LOSOutput, PathSegment
from ugv_control.utils import wrap_angle


class LOSGuidance:
    """
    Line-of-sight path-following guidance.

    Implements the main geometry from Section 3.1.2:
    - path angle alpha_k
    - along-track distance s
    - cross-track error e
    - total speed U
    - course angle chi
    - sideslip angle beta
    - desired heading psi_d
    - desired yaw rate r_d
    """

    def __init__(self, lookahead_distance: float) -> None:
        if lookahead_distance <= 0.0:
            raise ValueError("lookahead_distance must be positive")
        self.delta = lookahead_distance

    def compute(self, state: VehicleState, segment: PathSegment) -> LOSOutput:
        dx_path = segment.x_k1 - segment.x_k
        dy_path = segment.y_k1 - segment.y_k

        alpha_k = math.atan2(dy_path, dx_path)

        dx = state.x_n - segment.x_k
        dy = state.y_n - segment.y_k

        # Eq. (22)-(24)
        s = dx * math.cos(alpha_k) + dy * math.sin(alpha_k)
        e = -dx * math.sin(alpha_k) + dy * math.cos(alpha_k)

        # Eq. (19)
        U = math.sqrt(state.u * state.u + state.v * state.v)

        # Eq. (20), using atan2 for numerical robustness
        x_dot_n = state.u * math.cos(state.psi) - state.v * math.sin(state.psi)
        y_dot_n = state.u * math.sin(state.psi) + state.v * math.cos(state.psi)
        chi = math.atan2(y_dot_n, x_dot_n)

        # Eq. (30)
        if U > 1e-9:
            ratio = max(-1.0, min(1.0, state.v / U))
            beta = math.asin(ratio)
        else:
            beta = 0.0

        # Eq. (25)-(27)
        e_dot = -x_dot_n * math.sin(alpha_k) + y_dot_n * math.cos(alpha_k)

        # LOS correction
        los_correction = math.atan2(-e, self.delta)

        # Eq. (33)
        psi_d = wrap_angle(alpha_k + los_correction - beta)

        # Approximate beta_dot numerically-free simple version.
        # For a first implementation we set beta_dot = 0.
        # This matches the simplification later used in the paper experiments.
        beta_dot = 0.0

        # Eq. (41)
        r_d = -(self.delta / (e * e + self.delta * self.delta)) * e_dot - beta_dot

        return LOSOutput(
            alpha_k=alpha_k,
            s=s,
            e=e,
            U=U,
            chi=chi,
            beta=beta,
            psi_d=psi_d,
            r_d=r_d,
        )