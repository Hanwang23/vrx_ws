from pathlib import Path
import unittest

import yaml


RVIZ_CONFIG = Path(__file__).parents[1] / 'config' / 'pointcloud.rviz'
LIDAR_RVIZ_CONFIG = Path(__file__).parents[1] / 'config' / 'lidar_overview.rviz'


class RvizConfigTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        with RVIZ_CONFIG.open(encoding='utf-8') as stream:
            cls.config = yaml.safe_load(stream)

    def _display(self, display_class):
        displays = self.config['Visualization Manager']['Displays']
        return next(item for item in displays if item['Class'] == display_class)

    def test_marker_array_uses_controller_topic(self):
        marker_display = self._display('rviz_default_plugins/MarkerArray')
        self.assertNotIn('Marker Topic', marker_display)
        self.assertEqual(
            marker_display['Topic']['Value'],
            '/autonomous_usv/debug_markers',
        )

    def test_buoy_candidate_layer_is_visible_and_separate(self):
        marker_displays = [
            item for item in self.config['Visualization Manager']['Displays']
            if item['Class'] == 'rviz_default_plugins/MarkerArray'
        ]
        buoy_display = next(
            item for item in marker_displays
            if item['Topic']['Value'] == '/autonomous_usv/buoy_candidates')
        self.assertTrue(buoy_display['Enabled'])
        self.assertTrue(buoy_display['Value'])
        self.assertIn('浮标候选', buoy_display['Name'])

    def test_point_cloud_is_hidden_by_default_but_readable_when_enabled(self):
        cloud_display = self._display('rviz_default_plugins/PointCloud2')
        self.assertEqual(cloud_display['Color Transformer'], 'FlatColor')
        self.assertGreaterEqual(cloud_display['Alpha'], 0.5)
        self.assertGreaterEqual(cloud_display['Size (Pixels)'], 2)
        self.assertGreaterEqual(cloud_display['Decay Time'], 0.3)
        self.assertEqual(
            cloud_display['Topic']['Reliability Policy'], 'Best Effort')
        self.assertFalse(cloud_display['Enabled'])
        self.assertFalse(cloud_display['Value'])

    def test_planning_map_is_hidden_but_visible_enough_for_diagnostics(self):
        map_display = self._display('rviz_default_plugins/Map')
        self.assertGreaterEqual(map_display['Alpha'], 0.2)
        self.assertLessEqual(map_display['Alpha'], 0.35)
        self.assertFalse(map_display['Enabled'])
        self.assertFalse(map_display['Value'])
        background = self.config['Visualization Manager'][
            'Global Options']['Background Color']
        channels = [int(value.strip()) for value in background.split(';')]
        self.assertGreater(channels[2], channels[0])

    def test_raw_2d_lidar_is_a_high_contrast_default_layer(self):
        scan_display = self._display('rviz_default_plugins/LaserScan')
        self.assertTrue(scan_display['Enabled'])
        self.assertTrue(scan_display['Value'])
        self.assertEqual(
            scan_display['Topic']['Value'],
            '/wamv/sensors/lidars/lidar_wamv_sensor/scan',
        )
        self.assertEqual(
            scan_display['Topic']['Reliability Policy'], 'Best Effort')
        self.assertEqual(scan_display['Color Transformer'], 'FlatColor')
        self.assertGreaterEqual(scan_display['Size (Pixels)'], 3)
        red, green, blue = [
            int(value.strip()) for value in scan_display['Color'].split(';')]
        self.assertGreater(green, red)
        self.assertGreater(green, blue)

    def test_default_view_is_centered_and_readable(self):
        view = self.config['Visualization Manager']['Views']['Current']
        self.assertEqual(view['Class'], 'rviz_default_plugins/TopDownOrtho')
        self.assertGreaterEqual(view['Scale'], 10)
        self.assertLessEqual(view['Scale'], 16)
        self.assertGreater(view['X'], 8)
        self.assertLess(abs(view['Y']), 1.0)

    def test_mission_and_health_namespaces_have_clear_defaults(self):
        marker_display = self._display('rviz_default_plugins/MarkerArray')
        namespaces = marker_display['Namespaces']
        for name in (
            'mission_route', 'mission_waypoints', 'navigation_limits',
            'motion_vectors', 'sensor_health', 'planner_health',
        ):
            self.assertTrue(namespaces[name])
        self.assertFalse(namespaces['ilos_nominal_heading'])
        self.assertFalse(namespaces['target_direction'])
        self.assertFalse(namespaces['thrusters'])

    def test_default_window_prioritizes_the_visualization_canvas(self):
        geometry = self.config['Window Geometry']
        self.assertTrue(geometry['Hide Left Dock'])
        self.assertTrue(geometry['Hide Right Dock'])

    def test_local_view_keeps_the_only_tf_safe_fixed_frame(self):
        options = self.config['Visualization Manager']['Global Options']
        self.assertEqual(options['Fixed Frame'], 'wamv/wamv/base_link')

    def test_all_learning_launches_share_the_rich_rviz_config(self):
        launch_directory = RVIZ_CONFIG.parents[1] / 'launch'
        for filename in (
            'simulation.launch.py',
            'random_buoy_course.launch.py',
            'buoy_course.launch.py',
            'lattice_stress.launch.py',
            'colregs_learning.launch.py',
        ):
            with self.subTest(filename=filename):
                source = (launch_directory / filename).read_text(
                    encoding='utf-8')
                self.assertIn("'rviz_config'", source)
                self.assertIn("'pointcloud.rviz'", source)
                self.assertGreaterEqual(source.count("'rviz_config'"), 2)

    def test_lidar_overview_covers_full_sensor_range(self):
        with LIDAR_RVIZ_CONFIG.open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        displays = config['Visualization Manager']['Displays']
        scan = next(
            item for item in displays
            if item['Class'] == 'rviz_default_plugins/LaserScan')
        view = config['Visualization Manager']['Views']['Current']
        self.assertTrue(scan['Enabled'])
        self.assertEqual(scan['Topic']['Reliability Policy'], 'Best Effort')
        self.assertLessEqual(view['Scale'], 4.0)
        self.assertEqual(view['X'], 0)
        self.assertEqual(view['Y'], 0)


if __name__ == '__main__':
    unittest.main()
