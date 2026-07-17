"""Deterministic, collision-safe randomized buoy layouts for Wayfinding."""

import math
import random
from typing import Dict, List, Sequence, Tuple


BuoySpec = Tuple[str, str, float, float]

WAYPOINTS = (
    (-525.794, 171.500),
    (-550.984, 237.315),
    (-434.679, 179.863),
)
VESSEL_START = (-532.0, 162.0)
SAFE_OPERATING_BOUNDS = (-590.0, -400.0, 145.0, 275.0)
MIN_WAYPOINT_CLEARANCE_M = 10.0
MIN_START_CLEARANCE_M = 12.0


def _segment_frame(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    along = (dx / length, dy / length)
    left = (-along[1], along[0])
    return length, along, left


def _point_on_segment(start, along, left, distance, lateral):
    return (
        start[0] + distance * along[0] + lateral * left[0],
        start[1] + distance * along[1] + lateral * left[1],
    )


def minimum_pair_separation(specs: Sequence[BuoySpec]) -> float:
    if len(specs) < 2:
        return math.inf
    return min(
        math.hypot(first[2] - second[2], first[3] - second[3])
        for index, first in enumerate(specs)
        for second in specs[index + 1:]
    )


def minimum_point_clearance(
    specs: Sequence[BuoySpec],
    points: Sequence[Tuple[float, float]],
) -> float:
    if not specs or not points:
        return math.inf
    return min(
        math.hypot(spec[2] - point[0], spec[3] - point[1])
        for spec in specs
        for point in points
    )


def layout_spawn_issues(specs: Sequence[BuoySpec]) -> Tuple[str, ...]:
    issues = []
    if minimum_pair_separation(specs) < 7.0:
        issues.append('buoy pair separation is below 7 m')
    waypoint_clearance = minimum_point_clearance(specs, WAYPOINTS)
    if waypoint_clearance < MIN_WAYPOINT_CLEARANCE_M:
        issues.append('a buoy obstructs an official waypoint capture zone')
    start_clearance = minimum_point_clearance(specs, (VESSEL_START,))
    if start_clearance < MIN_START_CLEARANCE_M:
        issues.append('a buoy is too close to the WAM-V spawn pose')
    min_x, max_x, min_y, max_y = SAFE_OPERATING_BOUNDS
    if any(
        not (min_x <= spec[2] <= max_x and min_y <= spec[3] <= max_y)
        for spec in specs
    ):
        issues.append('a buoy is outside the validated open-water course bounds')
    return tuple(issues)


def generate_random_buoy_layout(seed: int) -> Tuple[BuoySpec, ...]:
    """Create six gates and four obstacles while retaining a feasible corridor."""
    for attempt in range(100):
        rng = random.Random(int(seed) * 1009 + attempt)
        specs: List[BuoySpec] = []
        gate_index = 0
        obstacle_index = 0
        for start, end in zip(WAYPOINTS, WAYPOINTS[1:]):
            length, along, left = _segment_frame(start, end)
            for nominal_fraction in (0.22, 0.50, 0.78):
                gate_index += 1
                fraction = nominal_fraction + rng.uniform(-0.025, 0.025)
                center_lateral = rng.uniform(-1.25, 1.25)
                half_width = rng.uniform(14.0, 18.0)
                center_distance = fraction * length
                red = _point_on_segment(
                    start, along, left, center_distance,
                    center_lateral + half_width)
                green = _point_on_segment(
                    start, along, left, center_distance,
                    center_lateral - half_width)
                specs.extend((
                    (f'random_gate_{gate_index}_red', 'red', *red),
                    (f'random_gate_{gate_index}_green', 'green', *green),
                ))

            for nominal_fraction in (0.35, 0.65):
                obstacle_index += 1
                fraction = nominal_fraction + rng.uniform(-0.025, 0.025)
                # A center-near obstacle forces active planning, while the wide
                # gate and open water preserve a realizable route on both sides.
                lateral = rng.uniform(-5.0, 5.0)
                point = _point_on_segment(
                    start, along, left, fraction * length, lateral)
                specs.append((
                    f'random_obstacle_{obstacle_index}', 'orange', *point))
        if not layout_spawn_issues(specs):
            return tuple(specs)
    raise RuntimeError(
        f'random layout seed {seed} could not satisfy spawn separation')


def layout_manifest(seed: int) -> Dict[str, object]:
    specs = generate_random_buoy_layout(seed)
    return {
        'scenario_seed': int(seed),
        'generator': 'wayfinding_random_v2',
        'waypoints_enu_m': [list(point) for point in WAYPOINTS],
        'vessel_start_enu_m': list(VESSEL_START),
        'safe_operating_bounds_enu_m': list(SAFE_OPERATING_BOUNDS),
        'minimum_pair_separation_m': minimum_pair_separation(specs),
        'minimum_waypoint_clearance_m': minimum_point_clearance(specs, WAYPOINTS),
        'minimum_start_clearance_m': minimum_point_clearance(
            specs, (VESSEL_START,)),
        'buoys': [
            {'name': name, 'color': color, 'x': x, 'y': y}
            for name, color, x, y in specs
        ],
    }
