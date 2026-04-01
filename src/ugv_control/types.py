from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class PathSegment:
    x_k: float
    y_k: float
    x_k1: float
    y_k1: float


@dataclass(slots=True)
class LOSOutput:
    alpha_k: float
    s: float
    e: float
    U: float
    chi: float
    beta: float
    psi_d: float
    r_d: float


@dataclass(slots=True)
class SurgeControlOutput:
    tau_x_c: float
    sigma_u: float
    u_tilde: float
    phi_x: float


@dataclass(slots=True)
class HeadingControlOutput:
    tau_psi_c: float
    sigma_r: float
    psi_tilde: float
    r_tilde: float
    phi_sigma_r: float