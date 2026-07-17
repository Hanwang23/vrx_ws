import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from codex_usv_controller.buoy_course import COLREGS_LEARNING_BUOYS
from codex_usv_controller.dynamic_map import mask_dynamic_scan_ranges
from codex_usv_controller.moving_target import target_pose
from codex_usv_controller.occupancy_grid import (
    OccupancyGridConfig,
    RollingOccupancyGrid,
)


class DynamicLearningTests(unittest.TestCase):
    def test_target_waits_then_moves_for_bounded_duration(self):
        waiting = target_pose(-500.0, 205.0, -1.0, 0.0, 10.0, 15.0, 90.0)
        moving = target_pose(-500.0, 205.0, -1.0, 0.0, 25.0, 15.0, 90.0)
        finished = target_pose(-500.0, 205.0, -1.0, 0.0, 200.0, 15.0, 90.0)
        self.assertEqual(-500.0, waiting.x)
        self.assertEqual(-510.0, moving.x)
        self.assertEqual(-590.0, finished.x)
        self.assertAlmostEqual(math.pi, moving.yaw)

    def test_dynamic_target_beam_is_masked_for_static_map_only(self):
        ranges = (10.0, 10.0, 10.0)
        masked, count = mask_dynamic_scan_ranges(
            0.0, 0.0, 0.0,
            ranges, -0.1, 0.1,
            ((10.0, 0.0),), 1.5)
        self.assertEqual(3, count)
        self.assertTrue(all(math.isnan(value) for value in masked))
        self.assertEqual((10.0, 10.0, 10.0), ranges)

    def test_nan_scan_does_not_clear_or_occupy_grid(self):
        grid = RollingOccupancyGrid(OccupancyGridConfig(
            width_m=20.0, height_m=20.0, resolution=1.0))
        grid.update_scan(
            0.0, 0.0, 0.0, (math.nan,), 0.0, 1.0, 0.1, 10.0, 1.0)
        snapshot = grid.snapshot(1.0)
        self.assertTrue(all(value == -1 for value in snapshot.probabilities))

    def test_learning_scene_has_six_visible_buoy_gates(self):
        self.assertEqual(12, len(COLREGS_LEARNING_BUOYS))
        colors = [spec[1] for spec in COLREGS_LEARNING_BUOYS]
        self.assertEqual(6, colors.count('red'))
        self.assertEqual(6, colors.count('green'))

    def test_dynamic_vessel_is_large_enough_for_visual_learning(self):
        root = Path(__file__).parents[1]
        model = ET.parse(
            root / 'models' / 'codex_target_vessel' / 'model.sdf').getroot()
        size = model.findtext(
            './model/link/collision/geometry/box/size').split()
        length, width, _height = (float(value) for value in size)
        self.assertGreaterEqual(length, 7.0)
        self.assertGreaterEqual(width, 2.8)
        self.assertIsNotNone(
            model.find("./model/link/visual[@name='port_navigation_light']"))
        self.assertIsNotNone(
            model.find("./model/link/visual[@name='starboard_navigation_light']"))

    def test_learning_launch_declares_second_target_and_buoys(self):
        root = Path(__file__).parents[1]
        source = (
            root / 'launch' / 'colregs_learning.launch.py'
        ).read_text(encoding='utf-8')
        self.assertIn("'spawn_second_target'", source)
        self.assertIn("'spawn_learning_buoys'", source)
        self.assertIn('COLREGS_LEARNING_BUOYS', source)


if __name__ == '__main__':
    unittest.main()
