from __future__ import annotations

import math

from ugv_control.models.vehicle_base import VehicleParams


_HDX_VARIANTS = {
    2350: {
        "tare_kg": 11650.0,
        "front_tare_kg": 7260.0,
        "rear_tare_kg": 4390.0,
        "turning_diameter_walls_m": 22.0,
        "overall_length_m": 8.583,
        "rear_overhang_m": 1.2,
    },
    2600: {
        "tare_kg": 11675.0,
        "front_tare_kg": 7325.0,
        "rear_tare_kg": 4350.0,
        "turning_diameter_walls_m": 22.8,
        "overall_length_m": 8.833,
        "rear_overhang_m": 1.2,
    },
    2850: {
        "tare_kg": 11705.0,
        "front_tare_kg": 7390.0,
        "rear_tare_kg": 4315.0,
        "turning_diameter_walls_m": 23.6,
        "overall_length_m": 9.083,
        "rear_overhang_m": 1.2,
    },
    3100: {
        "tare_kg": 11735.0,
        "front_tare_kg": 7450.0,
        "rear_tare_kg": 4285.0,
        "turning_diameter_walls_m": 24.3,
        "overall_length_m": 9.333,
        "rear_overhang_m": 1.2,
    },
    3600: {
        "tare_kg": 11835.0,
        "front_tare_kg": 7500.0,
        "rear_tare_kg": 4335.0,
        "turning_diameter_walls_m": 24.9,
        "overall_length_m": 10.133,
        "rear_overhang_m": 1.5,
    },
}

_AXLE12_M = 1.900
_AXLE34_M = 1.450
_TOTAL_WIDTH_M = 2.550
_FRONT_TRACK_EST_M = 2.050
_GVW_KG = 50000.0
_REAR_CAPACITY_KG = 32000.0
_FRONT_CAPACITY_STD_KG = 16000.0
_FRONT_CAPACITY_OPT_KG = 18000.0
_WHEEL_RADIUS_M = 0.525
_NUM_DRIVEN_WHEELS = 4
_MAX_DRIVE_TORQUE_PER_WHEEL_EST = 6000.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _yaw_inertia_box_estimate(mass_kg: float, length_m: float, width_m: float) -> float:
    return (mass_kg / 12.0) * (length_m**2 + width_m**2)


def _axle_positions_from_x(wheelbase_x_mm: int) -> list[float]:
    x23 = wheelbase_x_mm / 1000.0
    return [
        0.0,
        _AXLE12_M,
        _AXLE12_M + x23,
        _AXLE12_M + x23 + _AXLE34_M,
    ]


def _tandem_midpoints(axle_positions: list[float]) -> tuple[float, float]:
    front_mid = 0.5 * (axle_positions[0] + axle_positions[1])
    rear_mid = 0.5 * (axle_positions[2] + axle_positions[3])
    return front_mid, rear_mid


def _cg_from_group_reactions(
    front_group_load_kg: float,
    rear_group_load_kg: float,
    axle_positions: list[float],
) -> float:
    total = front_group_load_kg + rear_group_load_kg
    if total <= 0.0:
        raise ValueError("Total load must be positive.")

    front_mid, rear_mid = _tandem_midpoints(axle_positions)
    wheelbase_groups = rear_mid - front_mid
    if wheelbase_groups <= 0.0:
        raise ValueError("Rear tandem midpoint must be behind front tandem midpoint.")

    return front_mid + (rear_group_load_kg / total) * wheelbase_groups


def _estimate_loaded_group_split(
    payload_ratio: float,
    tare_kg: float,
    front_tare_kg: float,
    rear_tare_kg: float,
    use_optional_18t_front: bool,
) -> tuple[float, float, float]:
    """
    Interpolate front/rear axle-group loads from unloaded to fully loaded.

    payload_ratio:
        0.0 -> unloaded
        1.0 -> fully loaded
    """
    payload_ratio = _clamp(payload_ratio, 0.0, 1.0)

    payload_max_kg = _GVW_KG - tare_kg
    payload_kg = payload_ratio * payload_max_kg
    total_mass_kg = tare_kg + payload_kg

    front_full_kg = _FRONT_CAPACITY_OPT_KG if use_optional_18t_front else _FRONT_CAPACITY_STD_KG
    rear_full_kg = _REAR_CAPACITY_KG

    # Added load from tare to full-load estimate
    front_payload_share_kg = max(0.0, front_full_kg - front_tare_kg)
    rear_payload_share_kg = max(0.0, rear_full_kg - rear_tare_kg)
    total_payload_share_kg = front_payload_share_kg + rear_payload_share_kg

    if total_payload_share_kg <= 1e-9:
        front_group_kg = front_tare_kg
        rear_group_kg = rear_tare_kg + payload_kg
        return total_mass_kg, front_group_kg, rear_group_kg

    front_payload_fraction = front_payload_share_kg / total_payload_share_kg
    rear_payload_fraction = rear_payload_share_kg / total_payload_share_kg

    front_group_kg = front_tare_kg + front_payload_fraction * payload_kg
    rear_group_kg = rear_tare_kg + rear_payload_fraction * payload_kg

    return total_mass_kg, front_group_kg, rear_group_kg


def _steering_limits_from_turning_diameter(
    wheelbase_x_mm: int,
    turning_diameter_walls_m: float,
) -> tuple[float, float]:
    x23 = wheelbase_x_mm / 1000.0
    radius_m = 0.5 * turning_diameter_walls_m
    delta1 = math.atan2(x23, radius_m)
    delta1 *= 1.20
    delta1 = _clamp(delta1, 0.35, 0.55)
    delta2 = 0.75 * delta1
    return delta1, delta2


def _signed_axle_distances_from_cg(
    axle_positions: list[float],
    x_cg_from_axle1_m: float,
) -> tuple[float, float, float, float]:
    L1 = x_cg_from_axle1_m - axle_positions[0]
    L2 = x_cg_from_axle1_m - axle_positions[1]
    L3 = x_cg_from_axle1_m - axle_positions[2]
    L4 = x_cg_from_axle1_m - axle_positions[3]

    if not (L1 > 0.0 and L2 > 0.0 and L3 < 0.0 and L4 < 0.0):
        midpoint_23 = 0.5 * (axle_positions[1] + axle_positions[2])
        L1 = midpoint_23 - axle_positions[0]
        L2 = midpoint_23 - axle_positions[1]
        L3 = midpoint_23 - axle_positions[2]
        L4 = midpoint_23 - axle_positions[3]

    return L1, L2, L3, L4


def build_default_8ws_vehicle(
    wheelbase_x_mm: int = 2350,
    payload_ratio: float = 1.0,
    use_optional_18t_front: bool = True,
    desired_speed_mps: float = 5.0,
) -> VehicleParams:
    """
    Build an Astra HDX 8x4 parameter set from chassis X and payload ratio.

    payload_ratio:
        0.0 -> unloaded
        1.0 -> fully loaded
        0.5 -> half payload
    """
    if wheelbase_x_mm not in _HDX_VARIANTS:
        raise ValueError(
            f"Unsupported HDX wheelbase {wheelbase_x_mm}. "
            f"Choose one of {sorted(_HDX_VARIANTS.keys())}"
        )

    payload_ratio = _clamp(payload_ratio, 0.0, 1.0)

    variant = _HDX_VARIANTS[wheelbase_x_mm]
    axle_positions = _axle_positions_from_x(wheelbase_x_mm)

    tare_kg = variant["tare_kg"]
    front_tare_kg = variant["front_tare_kg"]
    rear_tare_kg = variant["rear_tare_kg"]

    m, front_group_kg, rear_group_kg = _estimate_loaded_group_split(
        payload_ratio=payload_ratio,
        tare_kg=tare_kg,
        front_tare_kg=front_tare_kg,
        rear_tare_kg=rear_tare_kg,
        use_optional_18t_front=use_optional_18t_front,
    )

    x_cg_from_axle1_m = _cg_from_group_reactions(
        front_group_load_kg=front_group_kg,
        rear_group_load_kg=rear_group_kg,
        axle_positions=axle_positions,
    )

    L1, L2, L3, L4 = _signed_axle_distances_from_cg(
        axle_positions=axle_positions,
        x_cg_from_axle1_m=x_cg_from_axle1_m,
    )

    max_steer_axle1, max_steer_axle2 = _steering_limits_from_turning_diameter(
        wheelbase_x_mm=wheelbase_x_mm,
        turning_diameter_walls_m=variant["turning_diameter_walls_m"],
    )

    I_z = _yaw_inertia_box_estimate(
        mass_kg=m,
        length_m=variant["overall_length_m"],
        width_m=_TOTAL_WIDTH_M,
    )

    # Smooth interpolation of simplified plant parameters with payload
    X_u_abs_u = -350.0 - 100.0 * payload_ratio
    yaw_time_constant = 2.5 + 0.3 * payload_ratio
    sway_time_constant = 3.0 + 0.3 * payload_ratio
    beta_gain = 0.35 - 0.03 * payload_ratio

    return VehicleParams(
        # Lumped vehicle parameters
        m=m,
        I_z=I_z,
        X_u_abs_u=X_u_abs_u,

        # Guidance
        lookahead_distance=25.0,
        acceptance_radius=12.0,

        # Surge controller
        k_x=0.07,
        k_x1=0.10,
        tau_x_max=0.65,

        # Heading controller
        k_r=2.0,
        k_psi=0.2,
        k_psi1=0.4,
        tau_psi_max=1.0,

        # Operating point
        desired_speed=desired_speed_mps,

        # 8x4 geometry
        L1=L1,
        L2=L2,
        L3=L3,
        L4=L4,
        track_width=_FRONT_TRACK_EST_M,

        # Steering
        max_steer_axle1=max_steer_axle1,
        max_steer_axle2=max_steer_axle2,

        # Drivetrain
        max_drive_torque_per_wheel=_MAX_DRIVE_TORQUE_PER_WHEEL_EST,
        num_driven_wheels=_NUM_DRIVEN_WHEELS,

        # Tire / wheel
        wheel_radius=_WHEEL_RADIUS_M,

        # Simplified plant parameters
        yaw_time_constant=yaw_time_constant,
        sway_time_constant=sway_time_constant,
        beta_gain=beta_gain,
    )