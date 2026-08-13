"""ROS-independent navigation, PID, avoidance, and thruster mixing logic."""

from dataclasses import dataclass
import math
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .guidance import DubinsPath, ILOSOutput, ILOSPathFollower, plan_dubins_path
from .occupancy_grid import OccupancySnapshot
from .state_lattice import DubinsStateLatticePlanner, StateLatticeConfig


EARTH_RADIUS_M = 6371000.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def validated_quaternion_yaw(
    x: float, y: float, z: float, w: float,
) -> Optional[float]:
    values = (float(x), float(y), float(z), float(w))
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 0.5 or norm > 1.5:
        return None
    x, y, z, w = (value / norm for value in values)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def geodetic_delta_m(
    from_latitude: float,
    from_longitude: float,
    to_latitude: float,
    to_longitude: float,
) -> Tuple[float, float]:
    """Return east and north offsets from one WGS84 point to another."""
    lat1 = math.radians(from_latitude)
    lat2 = math.radians(to_latitude)
    delta_lat = lat2 - lat1
    delta_lon = math.radians(to_longitude - from_longitude)
    mean_lat = 0.5 * (lat1 + lat2)
    east = EARTH_RADIUS_M * delta_lon * math.cos(mean_lat)
    north = EARTH_RADIUS_M * delta_lat
    return east, north


def enu_to_geodetic(
    origin_latitude: float,
    origin_longitude: float,
    east: float,
    north: float,
) -> Tuple[float, float]:
    """Convert a small local ENU offset back to WGS84 coordinates."""
    latitude = origin_latitude + math.degrees(north / EARTH_RADIUS_M)
    longitude_scale = EARTH_RADIUS_M * max(
        0.01, abs(math.cos(math.radians(origin_latitude))))
    longitude = origin_longitude + math.degrees(east / longitude_scale)
    return latitude, longitude


def distance_and_bearing(
    from_latitude: float,
    from_longitude: float,
    to_latitude: float,
    to_longitude: float,
) -> Tuple[float, float]:
    east, north = geodetic_delta_m(
        from_latitude, from_longitude, to_latitude, to_longitude)
    return math.hypot(east, north), math.atan2(north, east)


def _solve_3x3(matrix: Sequence[Sequence[float]], values: Sequence[float]):
    augmented = [list(row) + [float(value)] for row, value in zip(matrix, values)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-9:
            return None
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                current - factor * reference
                for current, reference in zip(augmented[row], augmented[column])
            ]
    return tuple(augmented[row][3] for row in range(3))


def _fit_height_plane(points: Sequence[Tuple[float, float, float]]):
    if len(points) < 12:
        return None
    count = float(len(points))
    sum_x = sum(point[0] for point in points)
    sum_y = sum(point[1] for point in points)
    sum_z = sum(point[2] for point in points)
    sum_xx = sum(point[0] * point[0] for point in points)
    sum_xy = sum(point[0] * point[1] for point in points)
    sum_yy = sum(point[1] * point[1] for point in points)
    sum_xz = sum(point[0] * point[2] for point in points)
    sum_yz = sum(point[1] * point[2] for point in points)
    return _solve_3x3(
        (
            (sum_xx, sum_xy, sum_x),
            (sum_xy, sum_yy, sum_y),
            (sum_x, sum_y, count),
        ),
        (sum_xz, sum_yz, sum_z),
    )


def extract_obstacle_points(
    points: Iterable[Tuple[float, float, float]],
    warning_distance: float,
    planning_angle: float,
    minimum_height: float = -1.75,
    maximum_height: float = 1.0,
    minimum_above_water: float = 0.10,
    cluster_cell_size: float = 0.45,
    cluster_min_points: int = 3,
) -> List[Tuple[float, float]]:
    """Remove the local water plane and return clustered polar obstacles."""
    samples: List[Tuple[float, float, float]] = []
    for raw_x, raw_y, raw_z in points:
        x, y, z = float(raw_x), float(raw_y), float(raw_z)
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        distance = math.hypot(x, y)
        if distance <= 0.3 or distance > warning_distance:
            continue
        if abs(math.atan2(y, x)) > planning_angle or z > maximum_height:
            continue
        samples.append((x, y, z))

    water_candidates = [point for point in samples if -2.8 <= point[2] <= -0.8]
    water_plane = _fit_height_plane(water_candidates)
    if water_plane is not None:
        a, b, c = water_plane
        inliers = [
            point for point in water_candidates
            if abs(point[2] - (a * point[0] + b * point[1] + c)) <= 0.18
        ]
        refined = _fit_height_plane(inliers)
        if refined is not None:
            water_plane = refined

    obstacle_samples: List[Tuple[float, float, float]] = []
    for x, y, z in samples:
        if water_plane is not None:
            a, b, c = water_plane
            if z - (a * x + b * y + c) < minimum_above_water:
                continue
        elif z < minimum_height:
            continue
        obstacle_samples.append((x, y, z))

    if not obstacle_samples:
        return []

    cell_size = max(0.1, cluster_cell_size)
    cells: Dict[Tuple[int, int], List[Tuple[float, float, float]]] = {}
    for point in obstacle_samples:
        key = (
            int(math.floor(point[0] / cell_size)),
            int(math.floor(point[1] / cell_size)),
        )
        cells.setdefault(key, []).append(point)

    obstacles: List[Tuple[float, float]] = []
    unvisited = set(cells)
    while unvisited:
        seed = unvisited.pop()
        pending = [seed]
        cluster = list(cells[seed])
        while pending:
            cell_x, cell_y = pending.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbor = (cell_x + dx, cell_y + dy)
                    if neighbor in unvisited:
                        unvisited.remove(neighbor)
                        pending.append(neighbor)
                        cluster.extend(cells[neighbor])
        if len(cluster) < max(1, int(cluster_min_points)):
            continue
        cluster.sort(key=lambda point: math.hypot(point[0], point[1]))
        surface = cluster[:max(1, min(5, len(cluster)))]
        x = sum(point[0] for point in surface) / len(surface)
        y = sum(point[1] for point in surface) / len(surface)
        obstacles.append((math.hypot(x, y), math.atan2(y, x)))
    return sorted(obstacles)


class PIDController:
    """PID with output limiting, conditional integration, and filtered D term."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float,
        integral_limit: float,
        derivative_alpha: float = 0.25,
    ) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = abs(float(output_limit))
        self.integral_limit = abs(float(integral_limit))
        self.derivative_alpha = clamp(float(derivative_alpha), 0.0, 1.0)
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error: Optional[float] = None
        self.filtered_derivative = 0.0

    def update(
        self,
        error: float,
        dt: float,
        angular: bool = False,
        output_limits: Optional[Tuple[float, float]] = None,
    ) -> float:
        if not math.isfinite(error) or not math.isfinite(dt) or dt <= 0.0:
            return 0.0

        dt = clamp(dt, 1e-3, 0.5)
        if self.previous_error is None:
            derivative = 0.0
        else:
            error_delta = error - self.previous_error
            if angular:
                error_delta = normalize_angle(error_delta)
            derivative = error_delta / dt
        self.previous_error = error
        self.filtered_derivative += self.derivative_alpha * (
            derivative - self.filtered_derivative)

        candidate_integral = clamp(
            self.integral + error * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        raw = (
            self.kp * error
            + self.ki * candidate_integral
            + self.kd * self.filtered_derivative
        )
        if output_limits is None:
            low_limit, high_limit = -self.output_limit, self.output_limit
        else:
            low_limit, high_limit = output_limits
            if low_limit > high_limit:
                low_limit, high_limit = high_limit, low_limit
        output = clamp(raw, low_limit, high_limit)

        saturated_high = raw > high_limit and error > 0.0
        saturated_low = raw < low_limit and error < 0.0
        if not saturated_high and not saturated_low:
            self.integral = candidate_integral
        return output


class GroundSpeedEstimator:
    """Low-pass horizontal speed estimate based on consecutive GNSS fixes."""

    def __init__(self, smoothing: float = 0.25, max_plausible_speed: float = 20.0):
        self.smoothing = clamp(smoothing, 0.0, 1.0)
        self.max_plausible_speed = max_plausible_speed
        self.previous: Optional[Tuple[float, float, float]] = None
        self.velocity_east = 0.0
        self.velocity_north = 0.0
        self.speed = 0.0

    def reset(self) -> None:
        self.previous = None
        self.velocity_east = 0.0
        self.velocity_north = 0.0
        self.speed = 0.0

    def update(self, latitude: float, longitude: float, timestamp: float) -> float:
        if self.previous is None:
            self.previous = (latitude, longitude, timestamp)
            return self.speed

        prev_lat, prev_lon, prev_time = self.previous
        self.previous = (latitude, longitude, timestamp)
        dt = timestamp - prev_time
        if dt <= 1e-3:
            return self.speed

        east, north = geodetic_delta_m(
            prev_lat, prev_lon, latitude, longitude)
        measured_east = east / dt
        measured_north = north / dt
        measured = math.hypot(measured_east, measured_north)
        if not math.isfinite(measured) or measured > self.max_plausible_speed:
            return self.speed
        self.velocity_east += self.smoothing * (
            measured_east - self.velocity_east)
        self.velocity_north += self.smoothing * (
            measured_north - self.velocity_north)
        self.speed = math.hypot(self.velocity_east, self.velocity_north)
        return self.speed


@dataclass(frozen=True)
class GeoTarget:
    latitude: float
    longitude: float
    yaw: Optional[float] = None


def nearest_neighbor_order(
    latitude: float,
    longitude: float,
    targets: Sequence[GeoTarget],
) -> List[GeoTarget]:
    """Return a greedy short route through unordered competition targets."""
    remaining = list(targets)
    ordered: List[GeoTarget] = []
    current_latitude = latitude
    current_longitude = longitude
    while remaining:
        index = min(
            range(len(remaining)),
            key=lambda candidate: distance_and_bearing(
                current_latitude,
                current_longitude,
                remaining[candidate].latitude,
                remaining[candidate].longitude,
            )[0],
        )
        target = remaining.pop(index)
        ordered.append(target)
        current_latitude = target.latitude
        current_longitude = target.longitude
    return ordered


@dataclass
class VesselState:
    latitude: float
    longitude: float
    yaw: float
    speed: float
    yaw_rate: float = 0.0
    yaw_rate_valid: bool = True
    laser_ranges: Sequence[float] = ()
    laser_angle_min: float = 0.0
    laser_angle_increment: float = 0.0
    laser_range_min: float = 0.1
    obstacle_points: Sequence[Tuple[float, float]] = ()
    east: Optional[float] = None
    north: Optional[float] = None
    occupancy_grid: Optional[OccupancySnapshot] = None
    colregs_active: bool = False
    colregs_heading_bias: float = 0.0
    colregs_speed_scale: float = 1.0
    colregs_action: str = 'none'


@dataclass
class ControlConfig:
    max_thrust: float = 1800.0
    max_surge_thrust: float = 500.0
    max_reverse_thrust: float = 500.0
    max_turn_thrust: float = 160.0
    max_low_speed_turn_thrust: float = 80.0
    turn_full_gain_speed: float = 1.2
    max_alignment_thrust: float = 60.0
    max_alignment_brake_thrust: float = 90.0
    alignment_heading_rate_gain: float = 0.35
    alignment_yaw_rate_gain: float = 350.0
    max_alignment_yaw_rate: float = math.radians(5.0)
    max_alignment_yaw_acceleration: float = math.radians(2.5)
    navigation_heading_rate_gain: float = 0.55
    navigation_yaw_rate_gain: float = 700.0
    max_navigation_yaw_rate: float = math.radians(12.0)
    max_navigation_yaw_acceleration: float = math.radians(5.0)
    yaw_rate_slowdown_start: float = math.radians(8.0)
    yaw_rate_slowdown_stop: float = math.radians(20.0)
    forward_thrust_sign: float = 1.0
    cruise_speed: float = 1.6
    minimum_approach_speed: float = 0.35
    approach_gain: float = 0.14
    position_tolerance: float = 2.0
    waypoint_exit_tolerance: float = 4.0
    station_exit_tolerance: float = 4.0
    heading_tolerance: float = math.radians(8.0)
    yaw_rate_tolerance: float = math.radians(3.0)
    heading_exit_tolerance: float = math.radians(12.0)
    yaw_rate_exit_tolerance: float = math.radians(8.0)
    waypoint_dwell_time: float = 1.0
    speed_feedforward: float = 260.0
    max_normal_brake_thrust: float = 240.0
    speed_brake_deadband: float = 0.20
    normal_brake_distance: float = 22.0
    guidance_enabled: bool = True
    dubins_turn_radius: float = 8.0
    dubins_sample_step: float = 0.5
    dubins_allow_three_turn_paths: bool = False
    ilos_lookahead: float = 8.0
    ilos_integral_gain: float = 0.015
    ilos_integral_limit: float = 3.0
    ilos_correction_limit: float = math.radians(45.0)
    ilos_integral_min_speed: float = 0.4
    curvature_feedforward_gain: float = 1.0
    max_lateral_acceleration: float = 0.12
    guidance_replan_path_deviation: float = 8.0
    guidance_replan_cooldown: float = 5.0
    lattice_enabled: bool = False
    lattice_heading_bins: int = 16
    lattice_planning_horizon: float = 40.0
    lattice_analytic_expansion_distance: float = 12.0
    lattice_max_expansions: int = 2500
    lattice_turn_penalty: float = 0.05
    lattice_replan_distance: float = 6.0
    lattice_blocked_path_confirmations: int = 2
    lattice_path_check_stride: int = 4
    lattice_start_clearance_radius: float = 4.0
    obstacle_warning_distance: float = 22.0
    obstacle_emergency_distance: float = 5.5
    obstacle_front_angle: float = math.radians(65.0)
    obstacle_planning_angle: float = math.radians(100.0)
    obstacle_safety_radius: float = 3.0
    obstacle_path_half_width: float = 2.4
    obstacle_bin_size: float = math.radians(3.0)
    obstacle_direction_hysteresis: float = math.radians(12.0)
    obstacle_cluster_range_tolerance: float = 0.8
    obstacle_brake_time_horizon: float = 2.5
    obstacle_emergency_time_horizon: float = 0.5
    obstacle_caution_speed: float = 0.55
    obstacle_brake_gain: float = 450.0
    obstacle_clear_hold_time: float = 0.5
    obstacle_stuck_timeout: float = 5.0
    obstacle_backup_duration: float = 2.0
    obstacle_backup_thrust: float = 350.0
    terminal_recovery_disable_radius: float = 8.0
    obstacle_avoidance_enabled: bool = True


def validate_control_config(config: ControlConfig) -> None:
    """Reject unsafe or internally inconsistent controller parameters."""
    positive = {
        'max_thrust': config.max_thrust,
        'max_surge_thrust': config.max_surge_thrust,
        'max_turn_thrust': config.max_turn_thrust,
        'max_low_speed_turn_thrust': config.max_low_speed_turn_thrust,
        'turn_full_gain_speed': config.turn_full_gain_speed,
        'max_alignment_thrust': config.max_alignment_thrust,
        'max_alignment_brake_thrust': config.max_alignment_brake_thrust,
        'alignment_heading_rate_gain': config.alignment_heading_rate_gain,
        'navigation_yaw_rate_gain': config.navigation_yaw_rate_gain,
        'max_navigation_yaw_acceleration': (
            config.max_navigation_yaw_acceleration),
        'alignment_yaw_rate_gain': config.alignment_yaw_rate_gain,
        'max_alignment_yaw_rate': config.max_alignment_yaw_rate,
        'max_alignment_yaw_acceleration': (
            config.max_alignment_yaw_acceleration),
        'position_tolerance': config.position_tolerance,
        'waypoint_exit_tolerance': config.waypoint_exit_tolerance,
        'obstacle_warning_distance': config.obstacle_warning_distance,
        'obstacle_emergency_distance': config.obstacle_emergency_distance,
        'obstacle_emergency_time_horizon': (
            config.obstacle_emergency_time_horizon),
        'obstacle_clear_hold_time': config.obstacle_clear_hold_time,
        'obstacle_caution_speed': config.obstacle_caution_speed,
        'dubins_turn_radius': config.dubins_turn_radius,
        'dubins_sample_step': config.dubins_sample_step,
        'ilos_lookahead': config.ilos_lookahead,
        'max_lateral_acceleration': config.max_lateral_acceleration,
        'guidance_replan_path_deviation': (
            config.guidance_replan_path_deviation),
        'lattice_planning_horizon': config.lattice_planning_horizon,
        'lattice_analytic_expansion_distance': (
            config.lattice_analytic_expansion_distance),
        'lattice_replan_distance': config.lattice_replan_distance,
        'lattice_start_clearance_radius': (
            config.lattice_start_clearance_radius),
        'max_normal_brake_thrust': config.max_normal_brake_thrust,
        'normal_brake_distance': config.normal_brake_distance,
        'terminal_recovery_disable_radius': (
            config.terminal_recovery_disable_radius),
    }
    invalid = [
        name for name, value in positive.items()
        if not math.isfinite(value) or value <= 0.0
    ]
    if invalid:
        raise ValueError(
            'Control parameters must be positive and finite: '
            + ', '.join(invalid))
    if config.max_surge_thrust > config.max_thrust:
        raise ValueError('max_surge_thrust must not exceed max_thrust')
    if config.max_normal_brake_thrust > config.max_reverse_thrust:
        raise ValueError(
            'normal brake thrust must not exceed max reverse thrust')
    if config.max_low_speed_turn_thrust > config.max_turn_thrust:
        raise ValueError(
            'low-speed turn thrust must not exceed max turn thrust')
    if config.position_tolerance >= config.waypoint_exit_tolerance:
        raise ValueError(
            'position_tolerance must be below waypoint_exit_tolerance')
    if config.heading_tolerance >= config.heading_exit_tolerance:
        raise ValueError(
            'heading_tolerance must be below heading_exit_tolerance')
    if config.yaw_rate_tolerance >= config.yaw_rate_exit_tolerance:
        raise ValueError(
            'yaw_rate_tolerance must be below yaw_rate_exit_tolerance')
    if config.obstacle_emergency_distance >= config.obstacle_warning_distance:
        raise ValueError(
            'obstacle_emergency_distance must be below warning_distance')
    if config.yaw_rate_slowdown_start >= config.yaw_rate_slowdown_stop:
        raise ValueError(
            'yaw-rate slowdown start must be below slowdown stop')
    if config.terminal_recovery_disable_radius < config.waypoint_exit_tolerance:
        raise ValueError(
            'terminal recovery radius must cover waypoint exit tolerance')
    if config.speed_brake_deadband < 0.0:
        raise ValueError('speed brake deadband must not be negative')
    if config.curvature_feedforward_gain < 0.0:
        raise ValueError('curvature feedforward gain must not be negative')
    if config.guidance_replan_cooldown < 0.0:
        raise ValueError('guidance replan cooldown must not be negative')
    if config.lattice_heading_bins < 8:
        raise ValueError('lattice heading bins must be at least eight')
    if config.lattice_max_expansions < 1:
        raise ValueError('lattice max expansions must be positive')
    if config.lattice_blocked_path_confirmations < 1:
        raise ValueError('lattice blocked-path confirmations must be positive')
    if config.lattice_path_check_stride < 1:
        raise ValueError('lattice path check stride must be positive')
    if config.lattice_turn_penalty < 0.0:
        raise ValueError('lattice turn penalty must not be negative')
    if config.guidance_enabled:
        dynamic_radius = config.cruise_speed / max(
            config.max_navigation_yaw_rate, math.radians(1.0))
        if config.dubins_turn_radius < 0.9 * dynamic_radius:
            raise ValueError(
                'dubins_turn_radius is below the navigation yaw-rate limit')


@dataclass
class ControlCommand:
    left_thrust: float = 0.0
    right_thrust: float = 0.0
    state: str = 'idle'
    target_index: int = 0
    target_count: int = 0
    distance: float = math.inf
    heading_error: float = 0.0
    desired_speed: float = 0.0
    nearest_obstacle: float = math.inf
    path_clearance: float = math.inf
    collision_clearance: float = math.inf
    guidance_mode: str = 'direct_los'
    path_valid: bool = False
    path_revision: int = 0
    path_segment_index: int = 0
    path_remaining: float = math.inf
    cross_track_error: float = 0.0
    path_deviation: float = 0.0
    nominal_heading_error: float = 0.0
    avoidance_override: bool = False
    avoidance_episode_active: bool = False
    guidance_replan_pending: bool = False
    guidance_replanned: bool = False
    guidance_replan_reason: str = ''
    guidance_replan_cooldown_remaining: float = 0.0
    ilos_integral_bias: float = 0.0
    path_curvature: float = 0.0
    upcoming_curvature: float = 0.0
    yaw_rate_feedforward: float = 0.0
    desired_yaw_rate: float = 0.0
    path_points_body: Tuple[Tuple[float, float], ...] = ()
    path_projection_body: Optional[Tuple[float, float]] = None
    lattice_expanded_states: int = 0
    lattice_map_revision: int = -1
    lattice_partial_path: bool = False
    lattice_fallback: bool = False
    lattice_blocked_confirmations: int = 0
    lattice_planning_time_ms: float = 0.0
    colregs_active: bool = False
    colregs_action: str = 'none'
    colregs_heading_bias: float = 0.0
    colregs_speed_scale: float = 1.0


@dataclass
class AvoidanceDecision:
    steering_angle: float
    speed_scale: float
    nearest_obstacle: float
    path_clearance: float
    collision_clearance: float
    no_gap: bool = False
    rear_clearance: float = math.inf


class ReactiveAvoidance:
    """Inflated polar-gap planner for local lidar obstacle avoidance."""

    def __init__(self) -> None:
        self.escape_direction = 0.0
        self.rear_turn_direction = 0.0
        self.clear_cycles = 0

    def reset(self) -> None:
        self.escape_direction = 0.0
        self.rear_turn_direction = 0.0
        self.clear_cycles = 0

    @staticmethod
    def _clearance(values: Iterable[float], range_min: float) -> float:
        valid = sorted(
            value for value in values
            if math.isfinite(value) and value >= max(0.0, range_min)
        )
        if not valid:
            return math.inf
        return valid[0]

    def compute(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        config: ControlConfig,
        range_min: float = 0.1,
        obstacle_points: Sequence[Tuple[float, float]] = (),
        target_angle: float = 0.0,
    ) -> AvoidanceDecision:
        """Choose a safe relative heading and evaluate its swept corridor."""
        planning_angle = max(
            config.obstacle_front_angle,
            min(math.pi, abs(config.obstacle_planning_angle)),
        )
        bin_size = max(math.radians(1.0), abs(config.obstacle_bin_size))
        bin_count = max(3, int(math.ceil(2.0 * planning_angle / bin_size)) + 1)
        bin_size = 2.0 * planning_angle / (bin_count - 1)
        raw_target_relative = normalize_angle(target_angle)
        if abs(raw_target_relative) > planning_angle:
            if self.rear_turn_direction == 0.0:
                self.rear_turn_direction = math.copysign(
                    1.0, raw_target_relative)
            target_relative = self.rear_turn_direction * planning_angle
        elif (
            self.rear_turn_direction != 0.0
            and abs(raw_target_relative)
            > planning_angle - config.obstacle_direction_hysteresis
        ):
            target_relative = self.rear_turn_direction * min(
                planning_angle, abs(raw_target_relative))
        else:
            self.rear_turn_direction = 0.0
            target_relative = raw_target_relative
        observations: List[List[float]] = [[] for _ in range(bin_count)]
        accepted: List[Tuple[float, float]] = []
        rear_observations: List[float] = []
        invalid_close_return = False

        def add_observation(distance: float, angle: float) -> None:
            if (
                not math.isfinite(distance)
                or distance < max(0.0, range_min)
                or distance > config.obstacle_warning_distance
            ):
                return
            angle = normalize_angle(angle)
            if abs(angle) >= math.pi - config.obstacle_front_angle:
                rear_observations.append(distance)
            if abs(angle) > planning_angle:
                return
            index = int(round((angle + planning_angle) / bin_size))
            index = int(clamp(index, 0, bin_count - 1))
            observations[index].append(distance)
            accepted.append((distance, angle))

        if (not ranges or angle_increment == 0.0) and not obstacle_points:
            self.clear_cycles += 1
            if self.clear_cycles >= 20:
                self.escape_direction = 0.0
            steering = target_relative
            return AvoidanceDecision(
                steering, 1.0, math.inf, math.inf, math.inf)

        if ranges and angle_increment != 0.0:
            cluster_tolerance = max(
                0.05, config.obstacle_cluster_range_tolerance)
            for index, raw_distance in enumerate(ranges):
                try:
                    distance = float(raw_distance)
                except (TypeError, ValueError, OverflowError):
                    continue
                too_close = bool(
                    distance == -math.inf
                    or (math.isfinite(distance) and distance < range_min))
                if too_close:
                    invalid_close_return = True
                    angle = normalize_angle(
                        angle_min + index * angle_increment)
                    add_observation(max(0.0, range_min), angle)
                    continue
                if not math.isfinite(distance):
                    continue
                support = 1
                for neighbor in (index - 1, index + 1):
                    if neighbor < 0 or neighbor >= len(ranges):
                        continue
                    neighbor_distance = ranges[neighbor]
                    if (
                        math.isfinite(neighbor_distance)
                        and abs(neighbor_distance - distance) <= cluster_tolerance
                    ):
                        support += 1
                if support < 2:
                    continue
                angle = normalize_angle(angle_min + index * angle_increment)
                add_observation(distance, angle)

        for distance, angle in obstacle_points:
            add_observation(distance, angle)

        clearance = [
            self._clearance(values, range_min) if values else math.inf
            for values in observations
        ]
        rear_clearance = self._clearance(rear_observations, range_min)
        front = [
            distance
            for index, distance in enumerate(clearance)
            if abs(-planning_angle + index * bin_size) <= config.obstacle_front_angle
        ]
        nearest = self._clearance(front, range_min)
        if all(not math.isfinite(distance) for distance in clearance):
            self.clear_cycles += 1
            if self.clear_cycles >= 20:
                self.escape_direction = 0.0
            steering = target_relative
            return AvoidanceDecision(
                steering, 1.0, nearest, math.inf, math.inf,
                rear_clearance=rear_clearance)
        self.clear_cycles = 0

        blocked = [False] * bin_count
        safety_radius = max(0.1, config.obstacle_safety_radius)
        for index, distance in enumerate(clearance):
            if not math.isfinite(distance):
                continue
            ratio = clamp(safety_radius / max(distance, 0.1), 0.0, 1.0)
            inflation = math.asin(ratio)
            inflation_bins = max(1, int(math.ceil(inflation / bin_size)))
            low = max(0, index - inflation_bins)
            high = min(bin_count - 1, index + inflation_bins)
            for blocked_index in range(low, high + 1):
                blocked[blocked_index] = True

        gaps: List[Tuple[int, int]] = []
        start: Optional[int] = None
        for index, is_blocked in enumerate(blocked + [True]):
            if not is_blocked and start is None:
                start = index
            elif is_blocked and start is not None:
                if index - start >= 2:
                    gaps.append((start, index - 1))
                start = None

        target_index = int(round((target_relative + planning_angle) / bin_size))
        target_index = int(clamp(target_index, 0, bin_count - 1))

        if not gaps:
            left_clearance = self._clearance(
                (
                    distance for index, distance in enumerate(clearance)
                    if index >= bin_count // 2
                ),
                range_min,
            )
            right_clearance = self._clearance(
                (
                    distance for index, distance in enumerate(clearance)
                    if index < bin_count // 2
                ),
                range_min,
            )
            recommended = 1.0 if left_clearance >= right_clearance else -1.0
            if self.escape_direction == 0.0:
                self.escape_direction = recommended
            elif recommended != self.escape_direction:
                current_clearance = (
                    left_clearance
                    if self.escape_direction > 0.0
                    else right_clearance
                )
                alternate_clearance = (
                    left_clearance if recommended > 0.0 else right_clearance)
                if (
                    math.isinf(alternate_clearance)
                    or alternate_clearance
                    > current_clearance + config.obstacle_safety_radius
                ):
                    self.escape_direction = recommended
            collision_clearance = self._corridor_clearance(
                accepted, 0.0, config.obstacle_path_half_width)
            return AvoidanceDecision(
                self.escape_direction * planning_angle,
                0.0,
                nearest,
                0.0,
                collision_clearance,
                no_gap=True,
                rear_clearance=rear_clearance,
            )

        left_clearance = self._clearance(
            (
                distance for index, distance in enumerate(clearance)
                if index > bin_count // 2
            ),
            range_min,
        )
        right_clearance = self._clearance(
            (
                distance for index, distance in enumerate(clearance)
                if index < bin_count // 2
            ),
            range_min,
        )
        preferred_direction = (
            self.escape_direction
            if self.escape_direction != 0.0
            else (1.0 if left_clearance >= right_clearance else -1.0)
        )

        best_angle = target_relative
        best_cost = math.inf
        for low, high in gaps:
            margin = 1 if high - low >= 2 else 0
            usable_low = low + margin
            usable_high = high - margin
            candidate_index = int(clamp(target_index, usable_low, usable_high))
            candidate_angle = -planning_angle + candidate_index * bin_size
            target_cost = abs(normalize_angle(candidate_angle - target_relative))
            candidate_direction = math.copysign(1.0, candidate_angle) if abs(
                candidate_angle) > 0.5 * bin_size else preferred_direction
            switch_cost = (
                config.obstacle_direction_hysteresis
                if candidate_direction != preferred_direction
                else 0.0
            )
            gap_width = (high - low + 1) * bin_size
            cost = target_cost + switch_cost - 0.03 * gap_width
            if cost < best_cost:
                best_cost = cost
                best_angle = candidate_angle

        deviation = normalize_angle(best_angle - target_relative)
        if abs(deviation) > bin_size:
            self.escape_direction = math.copysign(1.0, deviation)
        elif nearest >= config.obstacle_warning_distance:
            self.escape_direction = 0.0

        path_clearance = self._corridor_clearance(
            accepted, best_angle, config.obstacle_path_half_width)
        collision_clearance = self._corridor_clearance(
            accepted, 0.0, config.obstacle_path_half_width)
        span = max(
            0.1,
            config.obstacle_warning_distance - config.obstacle_emergency_distance,
        )
        clearance_ratio = clamp(
            (path_clearance - config.obstacle_emergency_distance) / span,
            0.0,
            1.0,
        )
        speed_scale = math.sqrt(clearance_ratio)
        if invalid_close_return:
            speed_scale = 0.0
        return AvoidanceDecision(
            best_angle,
            speed_scale,
            nearest,
            path_clearance,
            collision_clearance,
            rear_clearance=rear_clearance,
        )

    @staticmethod
    def _corridor_clearance(
        observations: Sequence[Tuple[float, float]],
        heading: float,
        half_width: float,
    ) -> float:
        clearance = math.inf
        corridor = max(0.1, half_width)
        for distance, angle in observations:
            relative = normalize_angle(angle - heading)
            longitudinal = distance * math.cos(relative)
            lateral = abs(distance * math.sin(relative))
            if longitudinal > 0.0 and lateral <= corridor:
                clearance = min(clearance, longitudinal)
        return clearance


class ControllerCore:
    """Task-state-independent closed-loop controller for an aft-thruster WAM-V."""

    def __init__(
        self,
        config: Optional[ControlConfig] = None,
        heading_pid: Optional[PIDController] = None,
        speed_pid: Optional[PIDController] = None,
    ) -> None:
        self.config = config or ControlConfig()
        self.heading_pid = heading_pid or PIDController(
            600.0, 18.0, 120.0,
            self.config.max_turn_thrust,
            math.radians(60.0),
        )
        self.speed_pid = speed_pid or PIDController(
            260.0, 35.0, 20.0,
            self.config.max_surge_thrust,
            8.0,
        )
        self.avoidance = ReactiveAvoidance()
        self.targets: List[GeoTarget] = []
        self.target_index = 0
        self.mode = 'wayfinding'
        self.dwell_elapsed = 0.0
        self.completed = False
        self.station_captured = False
        self.waypoint_captured = False
        self.heading_captured = False
        self.no_progress_elapsed = 0.0
        self.progress_anchor_distance: Optional[float] = None
        self.backup_remaining = 0.0
        self.collision_latched = False
        self.collision_clear_elapsed = 0.0
        self.alignment_active = False
        self.guidance_path: Optional[DubinsPath] = None
        self.guidance_origin: Optional[Tuple[float, float]] = None
        self.guidance_target_index = -1
        self.guidance_mode = 'direct_los'
        self.path_revision = 0
        self.guidance_avoidance_episode_active = False
        self.guidance_avoidance_clear_elapsed = 0.0
        self.guidance_replan_pending = False
        self.guidance_replan_pending_reason = ''
        self.guidance_replan_activation_pending = False
        self.guidance_replan_activation_reason = ''
        self.guidance_replan_reason = ''
        self.guidance_replan_cooldown_remaining = 0.0
        self.guidance_replanned_this_cycle = False
        self.guidance_path_changed_this_cycle = False
        self.guidance_origin_enu: Optional[Tuple[float, float]] = None
        self.guidance_partial_path = False
        self.terminal_guidance_active = False
        self.lattice_expanded_states = 0
        self.lattice_map_revision = -1
        self.lattice_fallback = False
        self.lattice_blocked_confirmations = 0
        self.lattice_planning_time_ms = 0.0
        self.lattice_last_checked_map_revision = -1
        self.lattice_planner = DubinsStateLatticePlanner(StateLatticeConfig(
            turn_radius=self.config.dubins_turn_radius,
            sample_step=self.config.dubins_sample_step,
            heading_bins=self.config.lattice_heading_bins,
            planning_horizon=self.config.lattice_planning_horizon,
            analytic_expansion_distance=(
                self.config.lattice_analytic_expansion_distance),
            max_expansions=self.config.lattice_max_expansions,
            turn_penalty=self.config.lattice_turn_penalty,
            start_clearance_radius=self.config.lattice_start_clearance_radius,
        ))
        self.navigation_yaw_rate_command = 0.0
        self.alignment_yaw_rate_command = 0.0
        self.ilos = ILOSPathFollower(
            lookahead=self.config.ilos_lookahead,
            integral_gain=self.config.ilos_integral_gain,
            integral_limit=self.config.ilos_integral_limit,
            correction_limit=self.config.ilos_correction_limit,
        )

    def _reset_recovery(self) -> None:
        self.no_progress_elapsed = 0.0
        self.progress_anchor_distance = None
        self.backup_remaining = 0.0
        self.collision_latched = False
        self.collision_clear_elapsed = 0.0

    def _reset_guidance(self) -> None:
        self.guidance_path = None
        self.guidance_origin = None
        self.guidance_origin_enu = None
        self.guidance_target_index = -1
        self.guidance_mode = 'direct_los'
        self.guidance_partial_path = False
        self.terminal_guidance_active = False
        self.lattice_blocked_confirmations = 0
        self.lattice_last_checked_map_revision = -1
        self._reset_guidance_replan_state()
        self.ilos.reset()

    def _cancel_guidance_replan_event(
        self, preserve_activation: bool = False,
    ) -> None:
        self.guidance_avoidance_episode_active = False
        self.guidance_avoidance_clear_elapsed = 0.0
        self.guidance_replan_pending = False
        self.guidance_replan_pending_reason = ''
        if not preserve_activation:
            if self.guidance_replan_activation_pending:
                self.guidance_path = None
                self.guidance_origin = None
                self.guidance_origin_enu = None
                self.guidance_target_index = -1
                self.guidance_mode = 'direct_los'
                self.guidance_partial_path = False
                self.ilos.reset()
            self.guidance_replan_activation_pending = False
            self.guidance_replan_activation_reason = ''
        self.guidance_replan_reason = ''
        self.guidance_replanned_this_cycle = False
        self.guidance_path_changed_this_cycle = False

    def _reset_guidance_replan_state(self) -> None:
        self._cancel_guidance_replan_event()
        self.guidance_replan_cooldown_remaining = 0.0

    def set_targets(self, targets: Sequence[GeoTarget], mode: str) -> None:
        self.targets = list(targets)
        self.target_index = 0
        self.mode = mode
        self.dwell_elapsed = 0.0
        self.completed = False
        self.station_captured = False
        self.waypoint_captured = False
        self.heading_captured = False
        self._reset_recovery()
        self._reset_guidance()
        self.alignment_active = False
        self.avoidance.reset()
        self.heading_pid.reset()
        self.speed_pid.reset()
        self.navigation_yaw_rate_command = 0.0
        self.alignment_yaw_rate_command = 0.0

    def stop(self, state: str = 'stopped') -> ControlCommand:
        self._reset_recovery()
        self._cancel_guidance_replan_event(preserve_activation=True)
        self.alignment_active = False
        self.heading_captured = False
        self.dwell_elapsed = 0.0
        self.avoidance.reset()
        self.heading_pid.reset()
        self.speed_pid.reset()
        self.navigation_yaw_rate_command = 0.0
        self.alignment_yaw_rate_command = 0.0
        return ControlCommand(
            state=state,
            target_index=self.target_index,
            target_count=len(self.targets),
            guidance_mode=self.guidance_mode,
            path_revision=self.path_revision,
            guidance_replan_pending=(
                self.guidance_replan_activation_pending),
            guidance_replan_cooldown_remaining=(
                self.guidance_replan_cooldown_remaining),
        )

    def update(self, vessel: VesselState, dt: float) -> ControlCommand:
        if self.completed:
            return self.stop('complete')
        if not self.targets or self.target_index >= len(self.targets):
            return self.stop('waiting_for_target')

        target = self.targets[self.target_index]
        distance, travel_bearing = distance_and_bearing(
            vessel.latitude,
            vessel.longitude,
            target.latitude,
            target.longitude,
        )
        target_yaw = target.yaw

        position_captured = distance <= self.config.position_tolerance
        if self.mode != 'stationkeeping':
            if position_captured:
                self.waypoint_captured = True
            elif (
                self.waypoint_captured
                and distance < self.config.waypoint_exit_tolerance
            ):
                position_captured = True
            else:
                self.waypoint_captured = False
                self.heading_captured = False

        if position_captured:
            heading_error = 0.0
            heading_ready = target_yaw is None
            if target_yaw is not None:
                heading_error = normalize_angle(target_yaw - vessel.yaw)
                strict_heading_ready = (
                    abs(heading_error) <= self.config.heading_tolerance
                    and vessel.yaw_rate_valid
                    and abs(vessel.yaw_rate) <= self.config.yaw_rate_tolerance
                )
                relaxed_heading_ready = (
                    self.heading_captured
                    and abs(heading_error)
                    <= self.config.heading_exit_tolerance
                    and vessel.yaw_rate_valid
                    and abs(vessel.yaw_rate)
                    <= self.config.yaw_rate_exit_tolerance
                )
                heading_ready = strict_heading_ready or relaxed_heading_ready
                if strict_heading_ready:
                    self.heading_captured = True
                elif not relaxed_heading_ready:
                    self.heading_captured = False
            if heading_ready:
                self.alignment_active = False
                if self.mode == 'stationkeeping':
                    self.station_captured = True
                    return self._stationary_command(
                        'stationkeeping', distance, heading_error)
                self.dwell_elapsed += max(0.0, dt)
                if self.dwell_elapsed >= self.config.waypoint_dwell_time:
                    self.target_index += 1
                    self.waypoint_captured = False
                    self.heading_captured = False
                    self.dwell_elapsed = 0.0
                    self._reset_recovery()
                    self._reset_guidance()
                    self.avoidance.reset()
                    self.heading_pid.reset()
                    self.speed_pid.reset()
                    self.navigation_yaw_rate_command = 0.0
                    self.alignment_yaw_rate_command = 0.0
                    if self.target_index >= len(self.targets):
                        self.completed = True
                        return self.stop('complete')
                return self._stationary_command('waypoint_dwell', distance, heading_error)
            self.dwell_elapsed = 0.0
            self.heading_captured = False
            self._begin_alignment()
            return self._heading_only_command(
                heading_error, vessel.yaw_rate, distance, dt, 'aligning',
                vessel)

        if (
            self.mode == 'stationkeeping'
            and self.station_captured
            and distance < self.config.station_exit_tolerance
        ):
            heading_error = (
                normalize_angle(target_yaw - vessel.yaw)
                if target_yaw is not None
                else 0.0
            )
            self._begin_alignment()
            return self._heading_only_command(
                heading_error, vessel.yaw_rate, distance, dt,
                'stationkeeping', vessel)

        if (
            self.mode == 'stationkeeping'
            and distance >= self.config.station_exit_tolerance
        ):
            self.station_captured = False

        self.dwell_elapsed = 0.0
        self.alignment_active = False
        self.heading_captured = False
        return self._navigation_command(
            vessel, target, travel_bearing, distance, dt)

    def _begin_alignment(self) -> None:
        if not self.alignment_active:
            self.heading_pid.reset()
            self._reset_recovery()
            self.navigation_yaw_rate_command = 0.0
            self.alignment_yaw_rate_command = 0.0
            self.alignment_active = True
        self._cancel_guidance_replan_event()

    def _stationary_command(
        self, state: str, distance: float, heading_error: float,
    ) -> ControlCommand:
        self.speed_pid.reset()
        self.heading_pid.reset()
        self.navigation_yaw_rate_command = 0.0
        self.alignment_yaw_rate_command = 0.0
        self._reset_recovery()
        self._cancel_guidance_replan_event()
        return ControlCommand(
            state=state,
            target_index=self.target_index,
            target_count=len(self.targets),
            distance=distance,
            heading_error=heading_error,
            guidance_mode=self.guidance_mode,
            path_revision=self.path_revision,
            guidance_replan_cooldown_remaining=(
                self.guidance_replan_cooldown_remaining),
        )

    def _heading_only_command(
        self,
        heading_error: float,
        yaw_rate: float,
        distance: float,
        dt: float,
        state: str,
        vessel: Optional[VesselState] = None,
    ) -> ControlCommand:
        self.speed_pid.reset()
        alignment_clearance = math.inf
        if vessel is not None and self.config.obstacle_avoidance_enabled:
            clearances = [
                value
                for value in vessel.laser_ranges
                if math.isfinite(value)
                and value >= max(0.0, vessel.laser_range_min)
            ]
            if any(value == -math.inf for value in vessel.laser_ranges):
                clearances.append(0.0)
            clearances.extend(
                float(point[0])
                for point in vessel.obstacle_points
                if math.isfinite(float(point[0]))
                and float(point[0]) >= 0.0
            )
            if clearances:
                alignment_clearance = min(clearances)
        alignment_blocked = (
            alignment_clearance
            <= self.config.obstacle_safety_radius + 1.0
        )
        alignment_limit = min(
            self.config.max_turn_thrust,
            self.config.max_alignment_thrust,
        )
        # Limit the requested yaw rate before closing the inner rate loop.
        # This keeps the low-damping hull from building enough angular
        # momentum to oscillate across the final waypoint orientation.
        self.heading_pid.reset()
        rate_limit = max(math.radians(0.5), self.config.max_alignment_yaw_rate)
        target_yaw_rate = (
            0.0
            if alignment_blocked
            else rate_limit * math.tanh(
                self.config.alignment_heading_rate_gain
                * heading_error / rate_limit)
        )
        max_rate_step = (
            self.config.max_alignment_yaw_acceleration
            * clamp(dt, 0.0, 0.5)
        )
        self.alignment_yaw_rate_command += clamp(
            target_yaw_rate - self.alignment_yaw_rate_command,
            -max_rate_step,
            max_rate_step,
        )
        desired_yaw_rate = self.alignment_yaw_rate_command
        raw_turn = self.config.alignment_yaw_rate_gain * (
            desired_yaw_rate - yaw_rate)
        turn_limit = alignment_limit
        if raw_turn * yaw_rate < 0.0:
            turn_limit = min(
                self.config.max_turn_thrust,
                max(alignment_limit, self.config.max_alignment_brake_thrust),
            )
        turn = clamp(raw_turn, -turn_limit, turn_limit)
        left, right = self._mix(0.0, turn)
        return ControlCommand(
            left_thrust=left,
            right_thrust=right,
            state='alignment_blocked' if alignment_blocked else state,
            target_index=self.target_index,
            target_count=len(self.targets),
            distance=distance,
            heading_error=heading_error,
            desired_yaw_rate=desired_yaw_rate,
            nearest_obstacle=alignment_clearance,
            collision_clearance=alignment_clearance,
            guidance_mode=self.guidance_mode,
            path_revision=self.path_revision,
            guidance_replan_cooldown_remaining=(
                self.guidance_replan_cooldown_remaining),
        )

    def _plan_guidance_path(
        self,
        vessel: VesselState,
        target: GeoTarget,
        travel_bearing: float,
        distance: float,
        increment_revision: bool = True,
        allow_fallback: bool = True,
    ) -> bool:
        planning_started = time.perf_counter()
        new_guidance_origin = (vessel.latitude, vessel.longitude)
        new_guidance_origin_enu = (
            (vessel.east, vessel.north)
            if vessel.east is not None and vessel.north is not None
            else None
        )
        goal_east, goal_north = geodetic_delta_m(
            vessel.latitude,
            vessel.longitude,
            target.latitude,
            target.longitude,
        )
        use_dubins = (
            self.config.guidance_enabled
            and target.yaw is not None
            and distance >= 2.0 * self.config.dubins_turn_radius
        )
        if use_dubins:
            lattice_plan = None
            lattice_requested = (
                self.config.lattice_enabled
                and vessel.occupancy_grid is not None
                and vessel.east is not None
                and vessel.north is not None
            )
            if lattice_requested:
                lattice_plan = self.lattice_planner.plan(
                    (vessel.east, vessel.north, vessel.yaw),
                    (
                        vessel.east + goal_east,
                        vessel.north + goal_north,
                        target.yaw,
                    ),
                    vessel.occupancy_grid,
                )
            if lattice_requested and lattice_plan is None and not allow_fallback:
                self.lattice_planning_time_ms += 1000.0 * (
                    time.perf_counter() - planning_started)
                return False
            if lattice_plan is not None:
                world_path = lattice_plan.path
                self.guidance_path = DubinsPath(
                    modes=world_path.modes,
                    segment_lengths=world_path.segment_lengths,
                    points=tuple(
                        (
                            point[0] - vessel.east,
                            point[1] - vessel.north,
                            point[2],
                        )
                        for point in world_path.points
                    ),
                    curvatures=world_path.curvatures,
                )
                self.guidance_partial_path = not lattice_plan.reached_goal
                self.lattice_expanded_states = lattice_plan.expanded_states
                self.lattice_map_revision = lattice_plan.map_revision
                self.lattice_last_checked_map_revision = lattice_plan.map_revision
                self.lattice_fallback = False
                self.guidance_mode = 'lattice_ilos'
            else:
                self.guidance_path = plan_dubins_path(
                    (0.0, 0.0, vessel.yaw),
                    (goal_east, goal_north, target.yaw),
                    self.config.dubins_turn_radius,
                    self.config.dubins_sample_step,
                    self.config.dubins_allow_three_turn_paths,
                )
                self.guidance_partial_path = False
                self.lattice_expanded_states = 0
                self.lattice_map_revision = (
                    vessel.occupancy_grid.revision
                    if vessel.occupancy_grid is not None else -1)
                self.lattice_last_checked_map_revision = self.lattice_map_revision
                self.lattice_fallback = self.config.lattice_enabled
                self.guidance_mode = 'dubins_ilos'
            points = tuple(
                (point[0], point[1]) for point in self.guidance_path.points)
            curvatures = self.guidance_path.curvatures
        else:
            self.guidance_path = None
            self.guidance_partial_path = False
            self.lattice_expanded_states = 0
            self.lattice_map_revision = -1
            self.lattice_last_checked_map_revision = -1
            self.lattice_fallback = False
            self.lattice_blocked_confirmations = 0
            points = ((0.0, 0.0), (goal_east, goal_north))
            curvatures = ()
            self.guidance_mode = (
                'ilos_line' if self.config.guidance_enabled else 'direct_los')
        self.guidance_origin = new_guidance_origin
        self.guidance_origin_enu = new_guidance_origin_enu
        self.ilos.reset(points, curvatures)
        self.guidance_target_index = self.target_index
        if increment_revision:
            self.path_revision += 1
        self.lattice_planning_time_ms += 1000.0 * (
            time.perf_counter() - planning_started)
        return True

    def _lattice_path_blocked(
        self, vessel: VesselState,
    ) -> Optional[bool]:
        grid = vessel.occupancy_grid
        if (
            self.guidance_mode != 'lattice_ilos'
            or grid is None
            or self.guidance_origin_enu is None
            or grid.revision <= self.lattice_last_checked_map_revision
        ):
            return None
        self.lattice_last_checked_map_revision = grid.revision
        start_index = min(
            max(0, self.ilos.segment_index + 1),
            max(0, len(self.ilos.points) - 1),
        )
        stride = max(1, self.config.lattice_path_check_stride)
        origin_east, origin_north = self.guidance_origin_enu
        future_points = self.ilos.points[start_index::stride]
        if self.ilos.points and (
            not future_points or future_points[-1] != self.ilos.points[-1]
        ):
            future_points = tuple(future_points) + (self.ilos.points[-1],)
        return any(
            grid.is_blocked(
                origin_east + point[0],
                origin_north + point[1],
            )
            for point in future_points
        )

    def _guidance_preview(
        self,
        vessel: VesselState,
        target: GeoTarget,
        travel_bearing: float,
        distance: float,
    ) -> Tuple[
        float,
        Optional[ILOSOutput],
        Tuple[Tuple[float, float], ...],
        Optional[Tuple[float, float]],
    ]:
        if not self.config.guidance_enabled:
            return travel_bearing, None, (), None
        if (
            self.guidance_target_index != self.target_index
            or self.guidance_origin is None
            or len(self.ilos.points) < 2
        ):
            self._plan_guidance_path(
                vessel, target, travel_bearing, distance)
        if (
            self.guidance_mode in ('dubins_ilos', 'lattice_ilos')
            and distance < 2.0 * self.config.dubins_turn_radius
        ):
            self._cancel_guidance_replan_event()
            self._plan_guidance_path(
                vessel, target, travel_bearing, distance)
            self.guidance_replanned_this_cycle = True
            self.guidance_replan_reason = 'terminal_approach'
            self.guidance_path_changed_this_cycle = True
            self.terminal_guidance_active = True
        elif (
            self.terminal_guidance_active
            and self.guidance_mode == 'ilos_line'
            and distance
            > 3.0 * self.config.dubins_turn_radius
        ):
            # Re-enter the collision-checked planner with an arrival heading
            # aimed at the waypoint. Planning directly to the final task yaw
            # can create a large Dubins loop just outside the capture circle;
            # final yaw is handled by the dedicated low-speed alignment loop.
            recovery_target = GeoTarget(
                target.latitude,
                target.longitude,
                travel_bearing,
            )
            replanned = self._plan_guidance_path(
                vessel,
                recovery_target,
                travel_bearing,
                distance,
                allow_fallback=False,
            )
            if replanned:
                self.terminal_guidance_active = False
                self.guidance_replanned_this_cycle = True
                self.guidance_replan_reason = 'terminal_recovery'
                self.guidance_path_changed_this_cycle = True
        assert self.guidance_origin is not None
        vessel_east, vessel_north = geodetic_delta_m(
            self.guidance_origin[0],
            self.guidance_origin[1],
            vessel.latitude,
            vessel.longitude,
        )
        output = self.ilos.preview(
            vessel_east,
            vessel_north,
            return_to_endpoint=not self.guidance_partial_path,
        )
        path_blocked = self._lattice_path_blocked(vessel)
        if path_blocked is True:
            self.lattice_blocked_confirmations += 1
        elif path_blocked is False:
            self.lattice_blocked_confirmations = 0
        if (
            self.lattice_blocked_confirmations
            >= self.config.lattice_blocked_path_confirmations
            and self.guidance_replan_cooldown_remaining <= 0.0
            and distance > max(
                2.0 * self.config.dubins_turn_radius,
                self.config.terminal_recovery_disable_radius,
            )
        ):
            replanned = self._plan_guidance_path(
                vessel,
                target,
                travel_bearing,
                distance,
                allow_fallback=False,
            )
            self.guidance_replan_cooldown_remaining = (
                self.config.guidance_replan_cooldown)
            self.lattice_blocked_confirmations = 0
            if replanned:
                self.guidance_replanned_this_cycle = True
                self.guidance_replan_reason = 'lattice_obstacle'
                self.guidance_path_changed_this_cycle = True
                assert self.guidance_origin is not None
                vessel_east, vessel_north = geodetic_delta_m(
                    self.guidance_origin[0],
                    self.guidance_origin[1],
                    vessel.latitude,
                    vessel.longitude,
                )
                output = self.ilos.preview(
                    vessel_east,
                    vessel_north,
                    return_to_endpoint=not self.guidance_partial_path,
                )
        if (
            self.guidance_partial_path
            and output.remaining_distance <= self.config.lattice_replan_distance
            and distance > self.config.lattice_replan_distance
            and self.guidance_replan_cooldown_remaining <= 0.0
        ):
            replanned = self._plan_guidance_path(
                vessel,
                target,
                travel_bearing,
                distance,
                allow_fallback=False,
            )
            if replanned:
                self.guidance_replan_cooldown_remaining = (
                    self.config.guidance_replan_cooldown)
                self.guidance_replanned_this_cycle = True
                self.guidance_replan_reason = 'lattice_horizon'
                self.guidance_path_changed_this_cycle = True
                assert self.guidance_origin is not None
                vessel_east, vessel_north = geodetic_delta_m(
                    self.guidance_origin[0],
                    self.guidance_origin[1],
                    vessel.latitude,
                    vessel.longitude,
                )
                output = self.ilos.preview(
                    vessel_east,
                    vessel_north,
                    return_to_endpoint=not self.guidance_partial_path,
                )

        cos_yaw = math.cos(vessel.yaw)
        sin_yaw = math.sin(vessel.yaw)

        def to_body(point: Tuple[float, float]) -> Tuple[float, float]:
            delta_east = point[0] - vessel_east
            delta_north = point[1] - vessel_north
            return (
                delta_east * cos_yaw + delta_north * sin_yaw,
                -delta_east * sin_yaw + delta_north * cos_yaw,
            )

        stride = max(1, len(self.ilos.points) // 120)
        path_points = tuple(to_body(point) for point in self.ilos.points[::stride])
        if path_points and path_points[-1] != to_body(self.ilos.points[-1]):
            path_points += (to_body(self.ilos.points[-1]),)
        return output.course, output, path_points, to_body(output.projection)

    def _navigation_command(
        self,
        vessel: VesselState,
        target: GeoTarget,
        travel_bearing: float,
        distance: float,
        dt: float,
    ) -> ControlCommand:
        self.lattice_planning_time_ms = 0.0
        self.guidance_path_changed_this_cycle = False
        self.guidance_replanned_this_cycle = (
            self.guidance_replan_activation_pending)
        if self.guidance_replan_activation_pending:
            self.path_revision += 1
            self.guidance_replan_reason = (
                self.guidance_replan_activation_reason)
            self.guidance_replan_activation_pending = False
            self.guidance_replan_activation_reason = ''
        else:
            self.guidance_replan_reason = ''
        self.guidance_replan_cooldown_remaining = max(
            0.0,
            self.guidance_replan_cooldown_remaining - max(0.0, dt),
        )
        (
            nominal_heading,
            guidance,
            path_points_body,
            path_projection_body,
        ) = self._guidance_preview(
            vessel, target, travel_bearing, distance)
        path_deviation = (
            math.hypot(*path_projection_body)
            if path_projection_body is not None
            else 0.0
        )
        preview_integral_bias = self.ilos.integral_bias
        if vessel.colregs_active:
            nominal_heading = normalize_angle(
                nominal_heading + vessel.colregs_heading_bias)
        desired_heading = nominal_heading
        speed_scale = clamp(vessel.colregs_speed_scale, 0.0, 1.0)
        nearest_obstacle = math.inf
        path_clearance = math.inf
        collision_clearance = math.inf
        avoidance = AvoidanceDecision(
            normalize_angle(nominal_heading - vessel.yaw),
            1.0,
            math.inf,
            math.inf,
            math.inf,
        )
        if self.config.obstacle_avoidance_enabled:
            # Obstacles beyond the stopping envelope behind a nearby goal must
            # not pull the vessel away from a waypoint it can already capture.
            obstacle_horizon = min(
                self.config.obstacle_warning_distance,
                distance
                + self.config.obstacle_safety_radius
                + max(0.0, vessel.speed)
                * self.config.obstacle_brake_time_horizon,
            )
            relevant_ranges = tuple(
                value
                if not math.isfinite(value) or value <= obstacle_horizon
                else math.inf
                for value in vessel.laser_ranges
            )
            relevant_points = tuple(
                point for point in vessel.obstacle_points
                if point[0] <= obstacle_horizon
            )
            target_relative = normalize_angle(nominal_heading - vessel.yaw)
            avoidance = self.avoidance.compute(
                relevant_ranges,
                vessel.laser_angle_min,
                vessel.laser_angle_increment,
                self.config,
                vessel.laser_range_min,
                relevant_points,
                target_relative,
            )
            speed_scale *= avoidance.speed_scale
            nearest_obstacle = avoidance.nearest_obstacle
            path_clearance = avoidance.path_clearance
            collision_clearance = avoidance.collision_clearance
            desired_heading = normalize_angle(
                vessel.yaw + avoidance.steering_angle)

        heading_error = normalize_angle(desired_heading - vessel.yaw)
        nominal_heading_error = normalize_angle(nominal_heading - vessel.yaw)
        sweep_clearances = [
            float(value)
            for value in vessel.laser_ranges
            if math.isfinite(float(value))
            and float(value) >= max(0.0, vessel.laser_range_min)
        ]
        sweep_clearances.extend(
            float(point[0])
            for point in vessel.obstacle_points
            if math.isfinite(float(point[0]))
            and float(point[0]) >= 0.0
        )
        turn_sweep_nearest = min(sweep_clearances, default=math.inf)
        turn_sweep_risk = (
            abs(heading_error) >= math.radians(45.0)
            and turn_sweep_nearest
            <= self.config.obstacle_safety_radius
            + self.config.obstacle_path_half_width
        )
        if turn_sweep_risk:
            nearest_obstacle = min(
                nearest_obstacle, turn_sweep_nearest)
            collision_clearance = min(
                collision_clearance, turn_sweep_nearest)
        avoidance_override = abs(normalize_angle(
            avoidance.steering_angle - nominal_heading_error
        )) > self.config.obstacle_bin_size
        heading_scale = clamp(math.cos(heading_error), 0.0, 1.0)
        if (
            self.guidance_mode == 'ilos_line'
            and distance < 2.0 * self.config.dubins_turn_radius
            and abs(heading_error) > math.radians(30.0)
        ):
            heading_scale *= heading_scale
        approach_metric = distance
        if guidance is not None:
            approach_metric = min(
                distance, max(0.0, guidance.remaining_distance))
        desired_speed = min(
            self.config.cruise_speed,
            max(
                self.config.minimum_approach_speed,
                approach_metric * self.config.approach_gain,
            ),
        )
        if guidance is not None and guidance.upcoming_curvature > 1e-5:
            curve_speed_limit = math.sqrt(
                self.config.max_lateral_acceleration
                / guidance.upcoming_curvature)
            desired_speed = min(desired_speed, curve_speed_limit)
        desired_speed *= heading_scale * speed_scale
        if vessel.yaw_rate_valid:
            yaw_span = max(
                math.radians(1.0),
                self.config.yaw_rate_slowdown_stop
                - self.config.yaw_rate_slowdown_start,
            )
            yaw_rate_scale = clamp(
                (
                    self.config.yaw_rate_slowdown_stop
                    - abs(vessel.yaw_rate)
                ) / yaw_span,
                0.0,
                1.0,
            )
            desired_speed *= yaw_rate_scale

        if self.config.obstacle_avoidance_enabled:
            progress_measure = (
                guidance.remaining_distance
                if guidance is not None
                else distance
            )
            recovery_obstacle_limit = (
                self.config.obstacle_emergency_distance
                + self.config.cruise_speed
                * self.config.obstacle_brake_time_horizon
            )
            recovery_has_obstacle = (
                avoidance.no_gap
                or collision_clearance <= recovery_obstacle_limit
            )
            recovery_allowed = (
                recovery_has_obstacle
                and distance > self.config.terminal_recovery_disable_radius
            )
            if not recovery_allowed:
                self.no_progress_elapsed = 0.0
                self.progress_anchor_distance = progress_measure
                if distance <= self.config.terminal_recovery_disable_radius:
                    self.backup_remaining = 0.0
            elif (
                self.progress_anchor_distance is None
                or progress_measure <= self.progress_anchor_distance - 0.5
            ):
                self.progress_anchor_distance = progress_measure
                self.no_progress_elapsed = 0.0
            else:
                self.no_progress_elapsed += max(0.0, dt)

            if recovery_allowed and self.backup_remaining <= 0.0 and (
                self.no_progress_elapsed >= self.config.obstacle_stuck_timeout
                and (abs(vessel.speed) < 0.35 or avoidance.no_gap)
            ):
                self.backup_remaining = self.config.obstacle_backup_duration
                self.no_progress_elapsed = 0.0
                self.progress_anchor_distance = progress_measure
        else:
            self._reset_recovery()

        state = 'navigating'
        caution_active = False
        backup_active_this_cycle = self.backup_remaining > 0.0
        if self.backup_remaining > 0.0:
            self.speed_pid.reset()
            rear_blocked = avoidance.rear_clearance <= (
                self.config.obstacle_safety_radius + 1.0)
            if rear_blocked:
                self.backup_remaining = 0.0
                surge = 0.0
                desired_speed = 0.0
                state = 'pivoting'
            else:
                self.backup_remaining = max(
                    0.0, self.backup_remaining - max(0.0, dt))
                surge = -min(
                    self.config.max_reverse_thrust,
                    self.config.obstacle_backup_thrust,
                )
                desired_speed = -0.4
                state = 'backing_away'
        else:
            caution_distance = (
                self.config.obstacle_emergency_distance
                + max(0.0, self.config.cruise_speed)
                * max(0.0, self.config.obstacle_brake_time_horizon)
            )
            hard_braking_distance = (
                self.config.obstacle_emergency_distance
                + max(0.0, vessel.speed)
                * max(0.0, self.config.obstacle_emergency_time_horizon)
            )
            caution_active = collision_clearance <= caution_distance
            measured_collision = (
                collision_clearance <= hard_braking_distance
                or turn_sweep_risk
            )
            if measured_collision:
                self.collision_latched = True
                self.collision_clear_elapsed = 0.0
            elif self.collision_latched:
                if collision_clearance > hard_braking_distance + 2.0:
                    self.collision_clear_elapsed += max(0.0, dt)
                    if (
                        self.collision_clear_elapsed
                        >= self.config.obstacle_clear_hold_time
                    ):
                        self.collision_latched = False
                        self.collision_clear_elapsed = 0.0
                else:
                    self.collision_clear_elapsed = 0.0
            collision_imminent = self.collision_latched
            if caution_active and not collision_imminent:
                desired_speed = min(
                    desired_speed, self.config.obstacle_caution_speed)
            if collision_imminent and abs(vessel.speed) > 0.15:
                self.speed_pid.reset()
                surge = -math.copysign(clamp(
                    self.config.obstacle_brake_gain * abs(vessel.speed),
                    0.0,
                    self.config.max_reverse_thrust,
                ), vessel.speed)
                desired_speed = 0.0
                state = 'braking'
            elif collision_imminent:
                self.speed_pid.reset()
                surge = 0.0
                desired_speed = 0.0
                state = 'pivoting'
            elif (
                desired_speed <= 0.05
                and vessel.speed <= self.config.speed_brake_deadband
            ):
                self.speed_pid.reset()
                surge = 0.0
                state = 'pivoting' if avoidance.no_gap else 'avoiding'
            else:
                speed_error = desired_speed - vessel.speed
                feedforward = self.config.speed_feedforward * desired_speed
                correction = self.speed_pid.update(
                    speed_error,
                    dt,
                    output_limits=(
                        -feedforward - self.config.max_normal_brake_thrust,
                        self.config.max_surge_thrust - feedforward,
                    ),
                )
                surge = feedforward + correction
                curve_braking_allowed = (
                    guidance is not None
                    and guidance.upcoming_curvature > 1e-5
                )
                normal_brake_allowed = (
                    approach_metric <= self.config.normal_brake_distance
                    or caution_active
                    or curve_braking_allowed
                )
                if (
                    vessel.speed
                    > desired_speed + self.config.speed_brake_deadband
                    and normal_brake_allowed
                ):
                    surge = clamp(
                        surge,
                        -self.config.max_normal_brake_thrust,
                        self.config.max_surge_thrust,
                    )
                else:
                    surge = clamp(surge, 0.0, self.config.max_surge_thrust)
                if surge < -1e-6:
                    state = (
                        'curve_braking'
                        if curve_braking_allowed
                        and approach_metric
                        > self.config.normal_brake_distance
                        else 'approach_braking'
                    )
                if caution_active or avoidance_override:
                    state = (
                        'approach_braking' if surge < -1e-6 else 'avoiding')
                elif vessel.colregs_active:
                    state = 'colregs_give_way'

        avoidance_active_now = (
            self.config.obstacle_avoidance_enabled
            and (
                avoidance_override
                or caution_active
                or self.collision_latched
                or avoidance.no_gap
                or backup_active_this_cycle
            )
        )
        avoidance_released = False
        if avoidance_active_now:
            self.guidance_avoidance_episode_active = True
            self.guidance_avoidance_clear_elapsed = 0.0
        elif self.guidance_avoidance_episode_active:
            self.guidance_avoidance_clear_elapsed += max(0.0, dt)
            if (
                self.guidance_avoidance_clear_elapsed
                >= self.config.obstacle_clear_hold_time
            ):
                self.guidance_avoidance_episode_active = False
                self.guidance_avoidance_clear_elapsed = 0.0
                avoidance_released = True

        if self.guidance_replan_pending:
            can_replan = (
                not avoidance_active_now
                and self.config.guidance_enabled
                and self.guidance_path is not None
                and self.guidance_mode in ('dubins_ilos', 'lattice_ilos')
                and self.guidance_target_index == self.target_index
                and target.yaw is not None
                and distance > 2.0 * self.config.dubins_turn_radius
            )
            if can_replan:
                replanned = self._plan_guidance_path(
                    vessel,
                    target,
                    travel_bearing,
                    distance,
                    increment_revision=False,
                    allow_fallback=False,
                )
                if replanned:
                    self.guidance_replan_cooldown_remaining = (
                        self.config.guidance_replan_cooldown)
                    self.guidance_replan_activation_pending = True
                    self.guidance_replan_activation_reason = (
                        self.guidance_replan_pending_reason)
                    self.guidance_path_changed_this_cycle = True
            self.guidance_replan_pending = False
            self.guidance_replan_pending_reason = ''

        if (
            avoidance_released
            and self.guidance_path is not None
            and self.guidance_mode in ('dubins_ilos', 'lattice_ilos')
            and self.guidance_target_index == self.target_index
            and not self.guidance_replan_pending
            and self.guidance_replan_cooldown_remaining <= 0.0
            and distance > 2.0 * self.config.dubins_turn_radius
            and path_deviation >= self.config.guidance_replan_path_deviation
        ):
            self.guidance_replan_pending = True
            self.guidance_replan_pending_reason = 'path_deviation'

        if guidance is not None:
            self.ilos.integrate(
                guidance.cross_track_error,
                dt,
                enabled=(
                    state == 'navigating'
                    and not avoidance_override
                    and not self.guidance_replanned_this_cycle
                    and not self.guidance_path_changed_this_cycle
                    and not vessel.colregs_active
                    and vessel.speed >= self.config.ilos_integral_min_speed
                ),
            )

        yaw_rate_feedforward = 0.0
        if (
            guidance is not None
            and state == 'navigating'
            and not avoidance_override
            and not vessel.colregs_active
            and desired_speed > 0.0
        ):
            yaw_rate_feedforward = (
                self.config.curvature_feedforward_gain
                * desired_speed
                * guidance.path_curvature
            )
        target_yaw_rate = clamp(
            self.config.navigation_heading_rate_gain * heading_error
            + yaw_rate_feedforward,
            -self.config.max_navigation_yaw_rate,
            self.config.max_navigation_yaw_rate,
        )
        if turn_sweep_risk:
            target_yaw_rate = 0.0
        if state == 'backing_away':
            direction = self.avoidance.escape_direction
            if direction == 0.0:
                direction = 1.0 if heading_error >= 0.0 else -1.0
            target_yaw_rate = direction * min(
                math.radians(8.0),
                self.config.max_navigation_yaw_rate,
            )
        max_rate_step = (
            self.config.max_navigation_yaw_acceleration
            * clamp(dt, 0.0, 0.5)
        )
        self.navigation_yaw_rate_command += clamp(
            target_yaw_rate - self.navigation_yaw_rate_command,
            -max_rate_step,
            max_rate_step,
        )
        desired_yaw_rate = self.navigation_yaw_rate_command
        turn_limit = self.config.max_low_speed_turn_thrust + (
            self.config.max_turn_thrust
            - self.config.max_low_speed_turn_thrust
        ) * clamp(
            max(0.0, desired_speed) / self.config.turn_full_gain_speed,
            0.0,
            1.0,
        )
        if vessel.yaw_rate_valid:
            turn = self.config.navigation_yaw_rate_gain * (
                desired_yaw_rate - vessel.yaw_rate)
            self.heading_pid.reset()
        else:
            turn = self.heading_pid.update(
                heading_error,
                dt,
                output_limits=(-turn_limit, turn_limit),
            )
        turn = clamp(
            turn,
            -turn_limit,
            turn_limit,
        )
        left, right = self._mix(surge, turn)
        return ControlCommand(
            left_thrust=left,
            right_thrust=right,
            state=state,
            target_index=self.target_index,
            target_count=len(self.targets),
            distance=distance,
            heading_error=heading_error,
            desired_speed=desired_speed,
            nearest_obstacle=nearest_obstacle,
            path_clearance=path_clearance,
            collision_clearance=collision_clearance,
            guidance_mode=(
                self.guidance_mode
                if self.config.guidance_enabled
                else 'direct_los'
            ),
            path_valid=guidance is not None,
            path_revision=self.path_revision,
            path_segment_index=(guidance.segment_index if guidance else 0),
            path_remaining=(
                guidance.remaining_distance if guidance else distance),
            cross_track_error=(
                guidance.cross_track_error if guidance else 0.0),
            path_deviation=path_deviation,
            nominal_heading_error=nominal_heading_error,
            avoidance_override=avoidance_override,
            avoidance_episode_active=(
                self.guidance_avoidance_episode_active),
            guidance_replan_pending=(
                self.guidance_replan_pending
                or self.guidance_replan_activation_pending),
            guidance_replanned=self.guidance_replanned_this_cycle,
            guidance_replan_reason=self.guidance_replan_reason,
            guidance_replan_cooldown_remaining=(
                self.guidance_replan_cooldown_remaining),
            ilos_integral_bias=(
                preview_integral_bias
                if self.guidance_path_changed_this_cycle
                else self.ilos.integral_bias
            ),
            path_curvature=(
                guidance.path_curvature if guidance else 0.0),
            upcoming_curvature=(
                guidance.upcoming_curvature if guidance else 0.0),
            yaw_rate_feedforward=yaw_rate_feedforward,
            desired_yaw_rate=desired_yaw_rate,
            path_points_body=path_points_body,
            path_projection_body=path_projection_body,
            lattice_expanded_states=self.lattice_expanded_states,
            lattice_map_revision=self.lattice_map_revision,
            lattice_partial_path=self.guidance_partial_path,
            lattice_fallback=self.lattice_fallback,
            lattice_blocked_confirmations=(
                self.lattice_blocked_confirmations),
            lattice_planning_time_ms=self.lattice_planning_time_ms,
            colregs_active=vessel.colregs_active,
            colregs_action=vessel.colregs_action,
            colregs_heading_bias=vessel.colregs_heading_bias,
            colregs_speed_scale=vessel.colregs_speed_scale,
        )

    def _mix(self, surge: float, turn: float) -> Tuple[float, float]:
        base = surge * math.copysign(1.0, self.config.forward_thrust_sign)
        # Positive turn is counter-clockwise. The differential sign follows
        # the configured forward polarity so tuning remains physically valid.
        polarity = math.copysign(1.0, self.config.forward_thrust_sign)
        left = base - polarity * turn
        right = base + polarity * turn
        peak = max(abs(left), abs(right), 1.0)
        if peak > self.config.max_thrust:
            scale = self.config.max_thrust / peak
            left *= scale
            right *= scale
        return left, right
