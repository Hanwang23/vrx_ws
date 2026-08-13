import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from han_usv_controller.buoy_course import BUOY_SPECS, LATTICE_STRESS_SPECS


class BuoyCourseTests(unittest.TestCase):
    WAYPOINTS = (
        (-525.794, 171.500),
        (-550.984, 237.315),
        (-434.679, 179.863),
    )

    @staticmethod
    def _distance_to_segment(point, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        projection = (
            (point[0] - start[0]) * dx
            + (point[1] - start[1]) * dy
        ) / length_squared
        projection = max(0.0, min(1.0, projection))
        closest = (
            start[0] + projection * dx,
            start[1] + projection * dy,
        )
        return math.hypot(point[0] - closest[0], point[1] - closest[1])

    @staticmethod
    def _signed_distance_to_line(point, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        return (
            dx * (point[1] - start[1])
            - dy * (point[0] - start[0])
        ) / math.hypot(dx, dy)

    def test_layout_has_six_gates_and_four_obstacles(self):
        names = [spec[0] for spec in BUOY_SPECS]
        colors = [spec[1] for spec in BUOY_SPECS]
        self.assertEqual(16, len(BUOY_SPECS))
        self.assertEqual(16, len(set(names)))
        self.assertEqual(6, colors.count('red'))
        self.assertEqual(6, colors.count('green'))
        self.assertEqual(4, colors.count('orange'))

    def test_buoys_do_not_overlap(self):
        for index, first in enumerate(BUOY_SPECS):
            for second in BUOY_SPECS[index + 1:]:
                distance = math.hypot(first[2] - second[2], first[3] - second[3])
                self.assertGreater(distance, 1.0, (first[0], second[0]))

    def test_orange_obstacles_leave_turning_clearance_from_route(self):
        segments = tuple(zip(self.WAYPOINTS, self.WAYPOINTS[1:]))
        for name, color, x, y in BUOY_SPECS:
            if color != 'orange':
                continue
            clearance = min(
                self._distance_to_segment((x, y), start, end)
                for start, end in segments
            )
            self.assertGreaterEqual(clearance, 11.5, name)

    def test_every_buoy_leaves_hull_clearance_from_direct_route(self):
        segments = tuple(zip(self.WAYPOINTS, self.WAYPOINTS[1:]))
        for name, _color, x, y in BUOY_SPECS:
            clearance = min(
                self._distance_to_segment((x, y), start, end)
                for start, end in segments
            )
            self.assertGreaterEqual(clearance, 9.5, name)

    def test_gate_pairs_straddle_their_route_segment(self):
        segments = tuple(zip(self.WAYPOINTS, self.WAYPOINTS[1:]))
        for gate_index in range(6):
            segment = segments[0 if gate_index < 3 else 1]
            red = BUOY_SPECS[2 * gate_index]
            green = BUOY_SPECS[2 * gate_index + 1]
            red_side = self._signed_distance_to_line(
                (red[2], red[3]), *segment)
            green_side = self._signed_distance_to_line(
                (green[2], green[3]), *segment)
            self.assertLess(red_side * green_side, 0.0, red[0])

    def test_models_are_static_and_collidable(self):
        root = Path(__file__).parents[1]
        model_dirs = (
            'han_marker_buoy_red',
            'han_marker_buoy_green',
            'han_round_buoy_orange',
        )
        for model_dir in model_dirs:
            model = ET.parse(root / 'models' / model_dir / 'model.sdf').getroot()
            self.assertEqual('true', model.findtext('./model/static'))
            self.assertIsNotNone(model.find('./model/link/collision/geometry'))
            self.assertGreaterEqual(len(model.findall('./model/link/visual')), 3)

    def test_lattice_stress_barrier_blocks_first_long_leg(self):
        first_leg = (self.WAYPOINTS[0], self.WAYPOINTS[1])
        clearances = [
            self._distance_to_segment((x, y), *first_leg)
            for _name, _color, x, y in LATTICE_STRESS_SPECS
        ]
        self.assertEqual(3, len(LATTICE_STRESS_SPECS))
        self.assertLess(min(clearances), 0.1)
        for first, second in zip(
            LATTICE_STRESS_SPECS, LATTICE_STRESS_SPECS[1:]
        ):
            spacing = math.hypot(first[2] - second[2], first[3] - second[3])
            self.assertAlmostEqual(spacing, 4.0, delta=0.05)

    def test_lattice_stress_launch_is_installed(self):
        launch_file = Path(__file__).parents[1] / 'launch' / 'lattice_stress.launch.py'
        self.assertTrue(launch_file.is_file())


if __name__ == '__main__':
    unittest.main()
