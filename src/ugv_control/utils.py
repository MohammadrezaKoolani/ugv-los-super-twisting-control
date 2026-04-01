from __future__ import annotations

import math


def wrap_angle(angle: float) -> float:
    """
    Wrap an angle to [-pi, pi].
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def sign(x: float) -> float:
    """
    Mathematical sign function.
    """
    if x > 0.0:
        return 1.0
    if x < 0.0:
        return -1.0
    return 0.0


def sat(value: float, limit: float) -> float:
    """
    Saturation function sat_limit(value).
    """
    if limit < 0.0:
        raise ValueError("limit must be non-negative")

    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def safe_sqrt_abs_signed(x: float) -> float:
    """
    Compute |x|^(1/2) * sign(x).
    """
    return math.sqrt(abs(x)) * sign(x)