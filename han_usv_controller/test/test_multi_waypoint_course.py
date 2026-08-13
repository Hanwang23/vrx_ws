import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from han_usv_controller.multi_waypoint_course import (
    SAFE_OPERATING_BOUNDS,
    VESSEL_START,
    WAYPOINTS,
    course_issues,
    course_manifest,
    course_turns,
    enu_to_geodetic,
    segment_lengths,
    write_course_world,
)


class MultiWaypointCourseTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).parents[1]

    def test_course_has_eight_safe_diverse_waypoints(self):
        self.assertEqual(8, len(WAYPOINTS))
        self.assertEqual((), course_issues())
        min_x, max_x, min_y, max_y = SAFE_OPERATING_BOUNDS
        for x, y, _yaw in WAYPOINTS:
            self.assertTrue(min_x <= x <= max_x)
            self.assertTrue(min_y <= y <= max_y)
        turns = course_turns()
        self.assertTrue(any(turn > math.radians(25.0) for turn in turns))
        self.assertTrue(any(turn < -math.radians(25.0) for turn in turns))

    def test_course_combines_short_and_long_planning_legs(self):
        lengths = segment_lengths()
        self.assertEqual(8, len(lengths))
        self.assertTrue(any(length < 16.0 for length in lengths))
        self.assertGreaterEqual(sum(length > 40.0 for length in lengths), 3)
        self.assertGreater(
            WAYPOINTS[-1][0], -565.0,
            'final waypoint must stay east of the observed southwest shoreline',
        )
        self.assertGreaterEqual(
            WAYPOINTS[-2][0], -550.0,
            'departure turn must stay east of the observed shoreline',
        )

    def test_gazebo_conversion_matches_original_known_waypoint(self):
        latitude, longitude, altitude = enu_to_geodetic(-525.794, 171.500)
        self.assertAlmostEqual(-33.7226766699, latitude, places=7)
        self.assertAlmostEqual(150.6740630167, longitude, places=7)
        self.assertLess(abs(altitude), 0.1)

    def test_generated_world_replaces_only_waypoint_count_and_keeps_name(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'eight_waypoints.sdf'
            write_course_world(
                self.root / 'worlds' / 'wayfinding_task.sdf', output)
            root = ET.parse(output).getroot()
            world = root.find('./world')
            self.assertIsNotNone(world)
            self.assertEqual('wayfinding_task', world.attrib['name'])
            poses = root.findall(
                ".//plugin[@name='vrx::WayfindingScoringPlugin']"
                '/waypoints/waypoint/pose')
            self.assertEqual(8, len(poses))
            self.assertTrue(all(len(pose.text.split()) == 3 for pose in poses))
            duration = float(root.findtext(
                ".//plugin[@name='vrx::WayfindingScoringPlugin']"
                '/running_state_duration'))
            self.assertGreater(duration, 1.0e8)

    def test_manifest_is_self_checking(self):
        manifest = course_manifest()
        self.assertEqual('multi_waypoint_stress_v4', manifest['scenario'])
        self.assertEqual(8, manifest['waypoint_count'])
        self.assertEqual(1, manifest['short_segment_count'])
        self.assertEqual(3, manifest['long_segment_count'])
        self.assertGreater(manifest['left_turn_count'], 0)
        self.assertGreater(manifest['right_turn_count'], 0)


if __name__ == '__main__':
    unittest.main()
