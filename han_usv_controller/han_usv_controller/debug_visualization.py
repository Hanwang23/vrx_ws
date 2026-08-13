"""Pure helpers for readable path-tracking debug visualization."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple


Color = Tuple[float, float, float]
TrackingSample = Tuple[float, float, bool]


@dataclass(frozen=True)
class TrackingStatistics:
    mean_abs_m: float | None
    max_abs_m: float | None
    sample_count: int


@dataclass(frozen=True)
class TrackingQuality:
    label: str
    color: Color


@dataclass(frozen=True)
class WaypointVisualState:
    label: str
    color: Color
    scale: float


def enu_offset_to_body(
    relative_east: float,
    relative_north: float,
    current_yaw: float,
) -> Tuple[float, float]:
    """Rotate one ENU offset into forward-left vessel coordinates."""
    cosine = math.cos(current_yaw)
    sine = math.sin(current_yaw)
    return (
        relative_east * cosine + relative_north * sine,
        -relative_east * sine + relative_north * cosine,
    )


def circle_points(
    radius: float,
    sample_count: int = 48,
) -> Tuple[Tuple[float, float], ...]:
    """Return a closed, evenly sampled circle for a LINE_STRIP marker."""
    radius = max(0.0, float(radius))
    sample_count = max(8, int(sample_count))
    return tuple(
        (
            radius * math.cos(2.0 * math.pi * step / sample_count),
            radius * math.sin(2.0 * math.pi * step / sample_count),
        )
        for step in range(sample_count + 1)
    )


def waypoint_visual_state(
    waypoint_index: int,
    active_index: int,
) -> WaypointVisualState:
    """Choose stable task colors for completed, active and pending goals."""
    if waypoint_index < active_index:
        return WaypointVisualState('DONE', (0.25, 0.95, 0.38), 0.85)
    if waypoint_index == active_index:
        return WaypointVisualState('CURRENT', (0.15, 0.55, 1.00), 1.35)
    return WaypointVisualState('PENDING', (0.72, 0.78, 0.86), 0.72)


def freshness_state(age_s: float | None, timeout_s: float) -> str:
    """Return a compact health label for one timestamp age."""
    if age_s is None or not math.isfinite(age_s):
        return 'WAIT'
    if age_s <= max(0.0, timeout_s):
        return 'OK'
    return 'STALE'


def enu_history_to_body(
    history: Iterable[Tuple[float, float]],
    current_east: float,
    current_north: float,
    current_yaw: float,
) -> Tuple[Tuple[float, float], ...]:
    """Express fixed-frame ENU history in the current vessel body frame."""
    points = []
    for east, north in history:
        relative_east = east - current_east
        relative_north = north - current_north
        points.append(enu_offset_to_body(
            relative_east, relative_north, current_yaw))
    return tuple(points)


def tracking_statistics(
    samples: Sequence[TrackingSample],
    now: float,
    window_s: float = 20.0,
) -> TrackingStatistics:
    """Summarize normal tracking, excluding deliberate safety deviations."""
    cutoff = now - max(0.1, window_s)
    errors = [
        abs(error)
        for timestamp, error, safety_active in samples
        if timestamp >= cutoff and not safety_active and math.isfinite(error)
    ]
    if not errors:
        return TrackingStatistics(None, None, 0)
    return TrackingStatistics(
        mean_abs_m=sum(errors) / len(errors),
        max_abs_m=max(errors),
        sample_count=len(errors),
    )


def filter_buoy_candidates(
    obstacle_points: Iterable[Tuple[float, float]],
    dynamic_points_body: Iterable[Tuple[float, float]],
    exclusion_radius: float,
    sensor_offset: Tuple[float, float] = (0.0, 0.0),
) -> Tuple[Tuple[float, float], ...]:
    """Keep confirmed lidar clusters that are not dedicated dynamic vessels."""
    dynamic_points = tuple(dynamic_points_body)
    radius = max(0.0, float(exclusion_radius))
    candidates = []
    for distance, angle in obstacle_points:
        if not math.isfinite(distance) or not math.isfinite(angle):
            continue
        forward = sensor_offset[0] + distance * math.cos(angle)
        left = sensor_offset[1] + distance * math.sin(angle)
        if any(
            math.hypot(forward - target[0], left - target[1]) <= radius
            for target in dynamic_points
        ):
            continue
        candidates.append((distance, angle))
    return tuple(candidates)


def stale_buoy_marker_ids(
    previous_count: int,
    current_count: int,
) -> Tuple[int, ...]:
    """Return marker IDs that must be removed after the candidate list shrinks."""
    first_stale_index = max(0, int(current_count)) + 1
    last_previous_index = max(0, int(previous_count))
    return tuple(
        marker_id
        for index in range(first_stale_index, last_previous_index + 1)
        for marker_id in (10 * index, 10 * index + 1, 10 * index + 2)
    )


def tracking_quality(
    path_valid: bool,
    current_abs_error_m: float,
    statistics: TrackingStatistics,
    safety_active: bool,
    terminal_active: bool = False,
) -> TrackingQuality:
    if not path_valid or not math.isfinite(current_abs_error_m):
        return TrackingQuality('NO ACTIVE PATH', (0.70, 0.75, 0.80))
    if terminal_active:
        return TrackingQuality('GOAL APPROACH', (0.20, 0.65, 1.00))
    if safety_active:
        return TrackingQuality('SAFETY MANEUVER', (1.00, 0.72, 0.08))
    recent_mean = statistics.mean_abs_m or 0.0
    if current_abs_error_m <= 1.0 and recent_mean <= 1.2:
        return TrackingQuality('ON TRACK', (0.20, 1.00, 0.38))
    if current_abs_error_m <= 2.5 and recent_mean <= 2.5:
        return TrackingQuality('RECOVERING', (1.00, 0.78, 0.12))
    return TrackingQuality('OFF PATH', (1.00, 0.22, 0.12))
