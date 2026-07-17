"""Helpers for keeping confirmed moving targets out of the static grid."""

import math
from typing import Iterable, Sequence, Tuple


def mask_dynamic_scan_ranges(
    sensor_east: float,
    sensor_north: float,
    yaw: float,
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    dynamic_targets: Iterable[Tuple[float, float]],
    mask_radius: float,
) -> Tuple[Tuple[float, ...], int]:
    """Replace beams ending on moving targets with NaN for grid fusion only."""
    targets = tuple(dynamic_targets)
    if not targets or mask_radius <= 0.0:
        return tuple(ranges), 0
    radius_squared = mask_radius * mask_radius
    masked = list(ranges)
    count = 0
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance <= 0.0:
            continue
        angle = yaw + angle_min + index * angle_increment
        endpoint_east = sensor_east + distance * math.cos(angle)
        endpoint_north = sensor_north + distance * math.sin(angle)
        if any(
            (endpoint_east - target_east) ** 2
            + (endpoint_north - target_north) ** 2
            <= radius_squared
            for target_east, target_north in targets
        ):
            masked[index] = math.nan
            count += 1
    return tuple(masked), count
