from __future__ import annotations

from ugv_control.models.states import DisturbanceEstimate, SurgeControllerConfig, VehicleParams, VehicleState
from ugv_control.types import SurgeControlOutput
from ugv_control.utils import safe_sqrt_abs_signed, sat


class SurgeSpeedController:
    """
    Super-twisting surge speed controller based on Section 3.1.1.
    """

    def __init__(self, config: SurgeControllerConfig) -> None:
        self.config = config
        self.tau_x1 = 0.0

    def reset(self) -> None:
        self.tau_x1 = 0.0

    def compute(
        self,
        state: VehicleState,
        params: VehicleParams,
        u_d: float,
        u_d_dot: float = 0.0,
        disturbance: DisturbanceEstimate | None = None,
        dt: float = 0.01,
    ) -> SurgeControlOutput:
        if disturbance is None:
            disturbance = DisturbanceEstimate()

        u_tilde = state.u - u_d

        # Eq. (18)
        phi_x = (
            -params.m * u_d_dot
            + params.m * state.v * state.r
            + params.X_u_abs_u * state.u * abs(state.u)
            + disturbance.d_x
        )

        # Use the closed-loop ideal relation to form sigma_u approximately.
        # sigma_u = m * u_tilde_dot + k_x |u_tilde|^(1/2) sgn(u_tilde)
        # Since u_tilde_dot is not always directly measured, we expose an approximate sigma_u
        # as phi_x + control_internal before saturation-like compensation.
        sqrt_term = safe_sqrt_abs_signed(u_tilde)

        # Eq. (16) internal update law
        raw_control = -self.config.k_x * sqrt_term + self.tau_x1
        tau_x_c = sat(raw_control, self.config.tau_x_max)

        if abs(tau_x_c) < self.config.tau_x_max:
            self.tau_x1 += -self.config.k_x1 * (1.0 if u_tilde > 0.0 else -1.0 if u_tilde < 0.0 else 0.0) * dt

        sigma_u = phi_x + tau_x_c

        return SurgeControlOutput(
            tau_x_c=tau_x_c,
            sigma_u=sigma_u,
            u_tilde=u_tilde,
            phi_x=phi_x,
        )