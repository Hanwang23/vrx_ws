import unittest

from codex_usv_controller.random_course import (
    MIN_START_CLEARANCE_M,
    MIN_WAYPOINT_CLEARANCE_M,
    SAFE_OPERATING_BOUNDS,
    VESSEL_START,
    WAYPOINTS,
    generate_random_buoy_layout,
    layout_spawn_issues,
    layout_manifest,
    minimum_pair_separation,
    minimum_point_clearance,
)


class RandomCourseTests(unittest.TestCase):
    def test_same_seed_is_reproducible(self):
        self.assertEqual(
            generate_random_buoy_layout(42),
            generate_random_buoy_layout(42))

    def test_different_seeds_change_layout(self):
        self.assertNotEqual(
            generate_random_buoy_layout(42),
            generate_random_buoy_layout(43))

    def test_thirty_reference_seeds_are_spawn_safe(self):
        for seed in range(1000, 1030):
            specs = generate_random_buoy_layout(seed)
            self.assertEqual(16, len(specs))
            self.assertEqual(16, len({spec[0] for spec in specs}))
            self.assertGreaterEqual(minimum_pair_separation(specs), 7.0)
            self.assertGreaterEqual(
                minimum_point_clearance(specs, WAYPOINTS),
                MIN_WAYPOINT_CLEARANCE_M)
            self.assertGreaterEqual(
                minimum_point_clearance(specs, (VESSEL_START,)),
                MIN_START_CLEARANCE_M)
            self.assertEqual((), layout_spawn_issues(specs))

    def test_manifest_records_seed_and_all_buoys(self):
        manifest = layout_manifest(1234)
        self.assertEqual(1234, manifest['scenario_seed'])
        self.assertEqual(16, len(manifest['buoys']))
        self.assertEqual(list(VESSEL_START), manifest['vessel_start_enu_m'])
        self.assertEqual(
            list(SAFE_OPERATING_BOUNDS),
            manifest['safe_operating_bounds_enu_m'])

    def test_layout_rejects_waypoint_and_spawn_obstructions(self):
        waypoint_obstruction = (
            ('bad_waypoint', 'orange', WAYPOINTS[0][0], WAYPOINTS[0][1]),)
        self.assertIn(
            'a buoy obstructs an official waypoint capture zone',
            layout_spawn_issues(waypoint_obstruction))
        spawn_obstruction = (
            ('bad_spawn', 'orange', VESSEL_START[0], VESSEL_START[1]),)
        self.assertIn(
            'a buoy is too close to the WAM-V spawn pose',
            layout_spawn_issues(spawn_obstruction))


if __name__ == '__main__':
    unittest.main()
