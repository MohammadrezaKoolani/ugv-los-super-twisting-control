from __future__ import annotations

import math

from ugv_control.models.vehicle_base import VehicleParams


# ---------------------------------------------------------------------
# Astra HDX 8x4 datasheet-backed chassis variants
# X is the spacing between axle 2 and axle 3 [mm]
# ---------------------------------------------------------------------
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

# ---------------------------------------------------------------------
# Fixed geometry visible in the HDX datasheet sketch
# ---------------------------------------------------------------------
_AXLE12_M = 1.900
_AXLE34_M = 1.450
_TOTAL_WIDTH_M = 2.550
_FRONT_TRACK_EST_M = 2.050  # front-view drawing width
_FRONT_OVERHANG_M = 1.550   # not directly used in dynamics

# ---------------------------------------------------------------------
# Capacities / estimates
# ---------------------------------------------------------------------
_GVW_KG = 50000.0
_FRONT_CAPACITY_STD_KG = 16000.0
_FRONT_CAPACITY_OPT_KG = 18000.0
_REAR_CAPACITY_KG = 32000.0

# Effective rolling radius estimate for 13R22.5
_WHEEL_RADIUS_M = 0.525

# Drivetrain: rear tandem driven, 4 driven wheels
_NUM_DRIVEN_WHEELS = 4

# Cursor 13 engine peak torque from datasheet: 2200/2300/2500 Nm
# Wheel torque is after transmission and axle reduction, so we keep this
# as a controller-side estimate for the simplified plant.
_MAX_DRIVE_TORQUE_PER_WHEEL_EST = 6000.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _yaw_inertia_box_estimate(mass_kg: float, length_m: float, width_m: float) -> float:
    """
    First-pass yaw inertia estimate around vertical axis.
    """
    return (mass_kg / 12.0) * (length_m**2 + width_m**2)


def _axle_positions_from_x(wheelbase_x_mm: int) -> list[float]:
    """
    Axle positions measured from axle 1 [m].
    """
    x23 = wheelbase_x_mm / 1000.0
    return [
        0.0,
        _AXLE12_M,
        _AXLE12_M + x23,
        _AXLE12_M + x23 + _AXLE34_M,
    ]


def _tandem_midpoints(axle_positions: list[float]) -> tuple[float, float]:
    """
    Midpoint of front tandem (axles 1,2) and rear tandem (axles 3,4).
    """
    front_mid = 0.5 * (axle_positions[0] + axle_positions[1])
    rear_mid = 0.5 * (axle_positions[2] + axle_positions[3])
    return front_mid, rear_mid


def _cg_from_group_reactions(
    front_group_load_kg: float,
    rear_group_load_kg: float,
    axle_positions: list[float],
) -> float:
    """
    Estimate longitudinal CG using front/rear tandem group reactions.
    Returns CG position measured from axle 1 [m].
    """
    total = front_group_load_kg + rear_group_load_kg
    if total <= 0.0:
        raise ValueError("Total load must be positive.")

    front_mid, rear_mid = _tandem_midpoints(axle_positions)
    wheelbase_groups = rear_mid - front_mid
    if wheelbase_groups <= 0.0:
        raise ValueError("Rear tandem midpoint must be behind front tandem midpoint.")

    # Static moment equilibrium about front tandem midpoint
    return front_mid + (rear_group_load_kg / total) * wheelbase_groups


def _estimate_loaded_group_split(
    use_optional_18t_front: bool,
) -> tuple[float, float]:
    """
    Estimate loaded front/rear tandem group loads from datasheet capacities.

    Standard front capacity gives 16t + 32t = 48t.
    Optional front capacity gives 18t + 32t = 50t, which matches GVW.
    """
    front_loaded = _FRONT_CAPACITY_OPT_KG if use_optional_18t_front else _FRONT_CAPACITY_STD_KG
    rear_loaded = _REAR_CAPACITY_KG
    return front_loaded, rear_loaded


def _steering_limits_from_turning_diameter(
    wheelbase_x_mm: int,
    turning_diameter_walls_m: float,
) -> tuple[float, float]:
    """
    Estimate front steering limits from wall-to-wall turning diameter.

    This is only an equivalent-control estimate for the simplified plant.
    It is not a full mechanical steering-geometry reconstruction.
    """
    x23 = wheelbase_x_mm / 1000.0
    radius_m = 0.5 * turning_diameter_walls_m

    # Use axle2-axle3 spacing as the primary steering wheelbase measure.
    delta1 = math.atan2(x23, radius_m)

    # Inflate slightly because real dual-front steering can achieve more
    # than a simple bicycle estimate suggests.
    delta1 *= 1.20

    # Keep within a realistic heavy-truck steering range
    delta1 = _clamp(delta1, 0.35, 0.55)

    # Second steering axle is usually smaller than first
    delta2 = 0.75 * delta1
    return delta1, delta2


def _signed_axle_distances_from_cg(
    axle_positions: list[float],
    x_cg_from_axle1_m: float,
) -> tuple[float, float, float, float]:
    """
    Return distances from CG in the sign convention used by your code:
    front axles positive, rear axles negative.
    """
    L1 = x_cg_from_axle1_m - axle_positions[0]
    L2 = x_cg_from_axle1_m - axle_positions[1]
    L3 = x_cg_from_axle1_m - axle_positions[2]
    L4 = x_cg_from_axle1_m - axle_positions[3]

    # Enforce front positive / rear negative. If estimate lands outside
    # axle2-axle3 interval, fall back to the midpoint between axle2 and axle3.
    if not (L1 > 0.0 and L2 > 0.0 and L3 < 0.0 and L4 < 0.0):
        midpoint_23 = 0.5 * (axle_positions[1] + axle_positions[2])
        L1 = midpoint_23 - axle_positions[0]
        L2 = midpoint_23 - axle_positions[1]
        L3 = midpoint_23 - axle_positions[2]
        L4 = midpoint_23 - axle_positions[3]

    return L1, L2, L3, L4


def build_default_8ws_vehicle(
    wheelbase_x_mm: int = 2350,
    loaded: bool = True,
    use_optional_18t_front: bool = True,
    desired_speed_mps: float = 5.0,
) -> VehicleParams:
    """
    Build an Astra HDX 8x4 parameter set from chassis X and load state.

    Parameters
    ----------
    wheelbase_x_mm:
        Chassis parameter X in mm. Must be one of:
        2350, 2600, 2850, 3100, 3600

    loaded:
        True  -> loaded truck estimate
        False -> unloaded truck estimate using datasheet tare split

    use_optional_18t_front:
        Only affects loaded=True.
        True  -> loaded split estimated from 18t front + 32t rear capacities
        False -> loaded split estimated from 16t front + 32t rear capacities

    desired_speed_mps:
        Controller operating speed.
    """
    if wheelbase_x_mm not in _HDX_VARIANTS:
        raise ValueError(
            f"Unsupported HDX wheelbase {wheelbase_x_mm}. "
            f"Choose one of {sorted(_HDX_VARIANTS.keys())}"
        )

    variant = _HDX_VARIANTS[wheelbase_x_mm]
    axle_positions = _axle_positions_from_x(wheelbase_x_mm)

    # ---------------------------------------------------------------
    # Mass and CG estimate
    # ---------------------------------------------------------------
    if loaded:
        m = _GVW_KG
        front_group_kg, rear_group_kg = _estimate_loaded_group_split(
            use_optional_18t_front=use_optional_18t_front
        )
    else:
        m = variant["tare_kg"]
        front_group_kg = variant["front_tare_kg"]
        rear_group_kg = variant["rear_tare_kg"]

    x_cg_from_axle1_m = _cg_from_group_reactions(
        front_group_load_kg=front_group_kg,
        rear_group_load_kg=rear_group_kg,
        axle_positions=axle_positions,
    )

    L1, L2, L3, L4 = _signed_axle_distances_from_cg(
        axle_positions=axle_positions,
        x_cg_from_axle1_m=x_cg_from_axle1_m,
    )

    # ---------------------------------------------------------------
    # Steering estimate from turning diameter
    # ---------------------------------------------------------------
    max_steer_axle1, max_steer_axle2 = _steering_limits_from_turning_diameter(
        wheelbase_x_mm=wheelbase_x_mm,
        turning_diameter_walls_m=variant["turning_diameter_walls_m"],
    )

    # ---------------------------------------------------------------
    # Yaw inertia estimate
    # ---------------------------------------------------------------
    I_z = _yaw_inertia_box_estimate(
        mass_kg=m,
        length_m=variant["overall_length_m"],
        width_m=_TOTAL_WIDTH_M,
    )

    # ---------------------------------------------------------------
    # Simplified-plant tuning
    # ---------------------------------------------------------------
    if loaded:
        X_u_abs_u = -450.0
        yaw_time_constant = 2.8
        sway_time_constant = 3.3
        beta_gain = 0.32
    else:
        X_u_abs_u = -350.0
        yaw_time_constant = 2.5
        sway_time_constant = 3.0
        beta_gain = 0.35

    return VehicleParams(
        # Lumped vehicle parameters
        m=m,
        I_z=I_z,
        X_u_abs_u=X_u_abs_u,

        # Guidance
        lookahead_distance=20.0,
        acceptance_radius=12.0,

        # Surge controller
        k_x=0.07,
        k_x1=0.10,
        tau_x_max=0.65,

        # Heading controller
        k_r=3.0,
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