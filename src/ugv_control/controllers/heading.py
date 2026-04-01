from __future__ import annotations

from ugv_control.models.states import DisturbanceEstimate, HeadingControllerConfig, VehicleParams, VehicleState
from ugv_control.types import HeadingControlOutput
from ugv_control.utils import safe_sqrt_abs_signed, sat, wrap_angle


class HeadingSuperTwistingController:
    """
    Super-twisting heading/yaw-rate controller based on Section 3.1.2.
    """

    def __init__(self, config: HeadingControllerConfig) -> None:
        self.config = config
        self.tau_psi1 = 0.0

    def reset(self) -> None:
        self.tau_psi1 = 0.0

    def compute(
        self,
        state: VehicleState,
        params: VehicleParams,
        psi_d: float,
        r_d: float,
        r_d_dot: float = 0.0,
        disturbance: DisturbanceEstimate | None = None,
        dt: float = 0.01,
    ) -> HeadingControlOutput:
        if disturbance is None:
            disturbance = DisturbanceEstimate()

        # Errors
        psi_tilde = wrap_angle(state.psi - psi_d)
        r_tilde = state.r - r_d

        # Eq. (44)
        sigma_r = psi_tilde + self.config.k_r * r_tilde

        sqrt_term = safe_sqrt_abs_signed(sigma_r)

        # Eq. (47)
        phi_sigma_r = (params.I_z / self.config.k_r) * r_tilde - params.I_z * r_d_dot + disturbance.d_psi

        # Eq. (44)
        raw_control = -self.config.k_psi * sqrt_term + self.tau_psi1
        tau_psi_c = sat(raw_control, self.config.tau_psi_max)

        if abs(tau_psi_c) < self.config.tau_psi_max:
            sign_sigma = 1.0 if sigma_r > 0.0 else -1.0 if sigma_r < 0.0 else 0.0
            self.tau_psi1 += -self.config.k_psi1 * sign_sigma * dt

        return HeadingControlOutput(
            tau_psi_c=tau_psi_c,
            sigma_r=sigma_r,
            psi_tilde=psi_tilde,
            r_tilde=r_tilde,
            phi_sigma_r=phi_sigma_r,
        )