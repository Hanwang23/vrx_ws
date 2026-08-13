"""Deterministic eight-waypoint stress course for controller regression."""

import json
import math
from pathlib import Path
from typing import Dict, Sequence, Tuple
import xml.etree.ElementTree as ET


WaypointSpec = Tuple[float, float, float]

WORLD_NAME = 'wayfinding_task'
SPHERICAL_ORIGIN = (-33.724223, 150.679736, 0.0)
VESSEL_START = (-532.0, 162.0)
SAFE_OPERATING_BOUNDS = (-590.0, -400.0, 145.0, 275.0)

# ENU x, ENU y and required final yaw. One leg is deliberately shorter than
# the terminal-planning radius; three long legs exercise lattice horizon
# handoff, while the turns and final headings cover both rotation directions.
WAYPOINTS: Tuple[WaypointSpec, ...] = (
    (-510.1, 168.4, math.radians(16.3)),
    (-505.4, 182.6, math.radians(71.7)),
    (-486.2, 195.5, math.radians(33.9)),
    (-480.5, 239.4, math.radians(82.6)),
    (-522.0, 235.1, math.radians(-174.1)),
    (-524.4, 264.0, math.radians(94.7)),
    (-550.0, 225.0, math.radians(-123.3)),
    (-545.0, 205.0, math.radians(-76.0)),
)


def segment_lengths(
    waypoints: Sequence[WaypointSpec] = WAYPOINTS,
    start: Tuple[float, float] = VESSEL_START,
) -> Tuple[float, ...]:
    points = (start,) + tuple((point[0], point[1]) for point in waypoints)
    return tuple(
        math.hypot(end[0] - begin[0], end[1] - begin[1])
        for begin, end in zip(points, points[1:])
    )


def course_turns(
    waypoints: Sequence[WaypointSpec] = WAYPOINTS,
    start: Tuple[float, float] = VESSEL_START,
) -> Tuple[float, ...]:
    points = (start,) + tuple((point[0], point[1]) for point in waypoints)
    bearings = tuple(
        math.atan2(end[1] - begin[1], end[0] - begin[0])
        for begin, end in zip(points, points[1:])
    )
    return tuple(
        (second - first + math.pi) % (2.0 * math.pi) - math.pi
        for first, second in zip(bearings, bearings[1:])
    )


def course_issues(
    waypoints: Sequence[WaypointSpec] = WAYPOINTS,
) -> Tuple[str, ...]:
    issues = []
    if len(waypoints) != 8:
        issues.append('the stress course must contain exactly eight waypoints')
    min_x, max_x, min_y, max_y = SAFE_OPERATING_BOUNDS
    if any(
        not (min_x <= x <= max_x and min_y <= y <= max_y)
        for x, y, _yaw in waypoints
    ):
        issues.append('a waypoint is outside the validated open-water bounds')
    points = tuple((x, y) for x, y, _yaw in waypoints)
    minimum_separation = min(
        (
            math.hypot(first[0] - second[0], first[1] - second[1])
            for index, first in enumerate(points)
            for second in points[index + 1:]
        ),
        default=math.inf,
    )
    if minimum_separation < 12.0:
        issues.append('two course waypoints are less than 12 m apart')
    turns = course_turns(waypoints)
    if not any(turn > math.radians(25.0) for turn in turns):
        issues.append('the course has no substantial left turn')
    if not any(turn < -math.radians(25.0) for turn in turns):
        issues.append('the course has no substantial right turn')
    if len({round(yaw, 1) for _x, _y, yaw in waypoints}) < 6:
        issues.append('the final-yaw commands are not sufficiently diverse')
    return tuple(issues)


def _spherical_converter():
    try:
        import gz.math7 as gz_math
    except ImportError as error:
        raise RuntimeError(
            'Gazebo Math 7 Python bindings are required to generate the '
            'multi-waypoint world') from error
    latitude = gz_math.Angle()
    latitude.set_degree(SPHERICAL_ORIGIN[0])
    longitude = gz_math.Angle()
    longitude.set_degree(SPHERICAL_ORIGIN[1])
    heading = gz_math.Angle()
    heading.set_degree(0.0)
    converter = gz_math.SphericalCoordinates(
        gz_math.SphericalCoordinates.EARTH_WGS84,
        latitude,
        longitude,
        SPHERICAL_ORIGIN[2],
        heading,
    )
    return gz_math, converter


def enu_to_geodetic(x: float, y: float) -> Tuple[float, float, float]:
    """Match Gazebo's LOCAL2-to-SPHERICAL conversion exactly."""
    gz_math, converter = _spherical_converter()
    spherical = converter.position_transform(
        gz_math.Vector3d(float(x), float(y), 0.0),
        gz_math.SphericalCoordinates.LOCAL2,
        gz_math.SphericalCoordinates.SPHERICAL,
    )
    return (
        math.degrees(spherical.x()),
        math.degrees(spherical.y()),
        spherical.z(),
    )


def write_course_world(
    template_path: Path,
    output_path: Path,
    waypoints: Sequence[WaypointSpec] = WAYPOINTS,
) -> Path:
    """Replace only the scoring-plugin waypoint block in an SDF template."""
    issues = course_issues(waypoints)
    if issues:
        raise ValueError('; '.join(issues))
    tree = ET.parse(str(template_path))
    root = tree.getroot()
    world = root.find('./world')
    if world is None:
        raise ValueError('template SDF does not contain a world element')
    if world.attrib.get('name') != WORLD_NAME:
        raise ValueError(
            f'template world must be named {WORLD_NAME!r}, got '
            f'{world.attrib.get("name")!r}')
    plugin = root.find(
        ".//plugin[@name='vrx::WayfindingScoringPlugin']")
    if plugin is None:
        raise ValueError('template SDF has no Wayfinding scoring plugin')
    waypoint_block = plugin.find('waypoints')
    if waypoint_block is None:
        raise ValueError('Wayfinding scoring plugin has no waypoints block')
    waypoint_block.clear()
    for x, y, yaw in waypoints:
        latitude, longitude, _altitude = enu_to_geodetic(x, y)
        waypoint = ET.SubElement(waypoint_block, 'waypoint')
        pose = ET.SubElement(waypoint, 'pose')
        pose.text = f'{latitude:.11f} {longitude:.11f} {yaw:.11f}'
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(output_path), encoding='utf-8', xml_declaration=True)
    return output_path


def course_manifest() -> Dict[str, object]:
    lengths = segment_lengths()
    return {
        'scenario': 'multi_waypoint_stress_v4',
        'world_name': WORLD_NAME,
        'waypoint_count': len(WAYPOINTS),
        'waypoints_enu_m': [
            {'x': x, 'y': y, 'yaw_rad': yaw}
            for x, y, yaw in WAYPOINTS
        ],
        'vessel_start_enu_m': list(VESSEL_START),
        'safe_operating_bounds_enu_m': list(SAFE_OPERATING_BOUNDS),
        'segment_lengths_m': list(lengths),
        'short_segment_count': sum(length < 16.0 for length in lengths),
        'long_segment_count': sum(length > 40.0 for length in lengths),
        'left_turn_count': sum(turn > 0.0 for turn in course_turns()),
        'right_turn_count': sum(turn < 0.0 for turn in course_turns()),
    }


def manifest_json() -> str:
    return json.dumps(course_manifest(), sort_keys=True)
