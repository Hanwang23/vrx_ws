import math
import unittest

from han_usv_controller.occupancy_grid import (
    OccupancyGridConfig,
    RollingOccupancyGrid,
    enu_grid_origin_in_body,
)


class RollingOccupancyGridTests(unittest.TestCase):
    def grid(self, **overrides):
        values = dict(
            width_m=20.0,
            height_m=20.0,
            resolution=1.0,
            max_range=8.0,
            ray_stride=1,
            stale_after=5.0,
            decay_rate=0.0,
        )
        values.update(overrides)
        return RollingOccupancyGrid(OccupancyGridConfig(**values))

    def test_scan_marks_free_ray_and_occupied_endpoint(self):
        grid = self.grid()
        for timestamp in (0.0, 0.1):
            grid.update_scan(
                0.0, 0.0, 0.0, [5.0], 0.0, 0.1,
                0.1, 10.0, timestamp)
        snapshot = grid.snapshot(0.1)
        self.assertGreater(snapshot.probabilities[
            snapshot.world_to_cell(5.0, 0.0)[1] * snapshot.width
            + snapshot.world_to_cell(5.0, 0.0)[0]], 50)
        self.assertLess(snapshot.probabilities[
            snapshot.world_to_cell(2.0, 0.0)[1] * snapshot.width
            + snapshot.world_to_cell(2.0, 0.0)[0]], 50)

    def test_infinite_return_only_marks_free_space(self):
        grid = self.grid()
        grid.update_scan(
            0.0, 0.0, 0.0, [math.inf], 0.0, 0.1,
            0.1, 8.0, 0.0)
        snapshot = grid.snapshot(0.0)
        endpoint = snapshot.world_to_cell(7.0, 0.0)
        self.assertLess(
            snapshot.probabilities[endpoint[1] * snapshot.width + endpoint[0]],
            50,
        )

    def test_negative_infinite_return_does_not_clear_unknown_space(self):
        grid = self.grid()
        grid.update_scan(
            0.0, 0.0, 0.0, [-math.inf], 0.0, 0.1,
            0.1, 8.0, 0.0)
        snapshot = grid.snapshot(0.0)
        self.assertTrue(all(value == -1 for value in snapshot.probabilities))

    def test_below_minimum_return_does_not_mark_a_free_ray(self):
        grid = self.grid()
        grid.update_scan(
            0.0, 0.0, 0.0, [0.05], 0.0, 0.1,
            0.1, 8.0, 0.0)
        snapshot = grid.snapshot(0.0)
        self.assertTrue(all(value == -1 for value in snapshot.probabilities))

    def test_recentering_preserves_world_obstacle(self):
        grid = self.grid(width_m=10.0, height_m=10.0)
        for timestamp in (0.0, 0.1):
            grid.update_scan(
                0.0, 0.0, 0.0, [3.0], 0.0, 0.1,
                0.1, 8.0, timestamp)
        grid.recenter(2.0, 0.0)
        snapshot = grid.snapshot(0.2)
        obstacle = snapshot.world_to_cell(3.0, 0.0)
        self.assertIsNotNone(obstacle)
        self.assertGreater(
            snapshot.probabilities[obstacle[1] * snapshot.width + obstacle[0]],
            50,
        )

    def test_stale_cells_return_to_unknown(self):
        grid = self.grid(stale_after=1.0)
        grid.update_scan(
            0.0, 0.0, 0.0, [3.0], 0.0, 0.1,
            0.1, 8.0, 0.0)
        snapshot = grid.snapshot(2.0)
        self.assertTrue(all(value == -1 for value in snapshot.probabilities))

    def test_long_sensor_gap_fully_decays_old_log_odds(self):
        grid = self.grid(decay_rate=0.08, stale_after=200.0)
        for timestamp in (0.0, 0.1):
            grid.update_scan(
                0.0, 0.0, 0.0, [3.0], 0.0, 0.1,
                0.1, 8.0, timestamp)
        obstacle = grid.world_to_cell(3.0, 0.0)
        self.assertGreater(grid.log_odds[grid._index(*obstacle)], 1.0)
        grid.snapshot(100.0)
        self.assertLess(abs(grid.log_odds[grid._index(*obstacle)]), 0.01)

    def test_inflation_blocks_neighboring_cells(self):
        grid = self.grid()
        for timestamp in (0.0, 0.1):
            grid.update_scan(
                0.0, 0.0, 0.0, [3.0], 0.0, 0.1,
                0.1, 8.0, timestamp)
        snapshot = grid.snapshot(0.1, inflation_radius=2.0)
        self.assertTrue(snapshot.is_blocked(3.0, 1.0))
        self.assertFalse(snapshot.is_blocked(0.0, 3.0))

    def test_enu_grid_origin_transforms_into_body_frame(self):
        x, y, yaw = enu_grid_origin_in_body(
            10.0, 20.0, 10.0, 10.0, math.pi / 2.0)
        self.assertAlmostEqual(x, 10.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(yaw, -math.pi / 2.0, places=6)

    def test_reset_discards_origin_and_all_observations(self):
        grid = self.grid()
        grid.update_scan(
            0.0, 0.0, 0.0, [3.0], 0.0, 0.1,
            0.1, 8.0, 0.0)
        self.assertGreater(grid.revision, 0)
        grid.reset()
        self.assertEqual(grid.revision, 0)
        self.assertIsNone(grid.origin_east)
        self.assertTrue(all(value == 0.0 for value in grid.log_odds))
        with self.assertRaises(RuntimeError):
            grid.snapshot(1.0)

    def test_confirmed_point_tracks_create_occupied_cells(self):
        grid = self.grid()
        grid.update_obstacles(0.0, 0.0, [(4.0, 2.0)], 0.0)
        grid.update_obstacles(0.0, 0.0, [(4.0, 2.0)], 0.1)
        snapshot = grid.snapshot(0.1)
        cell = snapshot.world_to_cell(4.0, 2.0)
        probability = snapshot.probabilities[
            cell[1] * snapshot.width + cell[0]]
        self.assertGreater(probability, 65)
        self.assertTrue(snapshot.is_blocked(4.0, 2.0))

    def test_scan_updates_each_free_cell_only_once_per_frame(self):
        grid = self.grid(ray_stride=1)
        grid.update_scan(
            0.0,
            0.0,
            0.0,
            [math.inf] * 21,
            -0.1,
            0.01,
            0.1,
            8.0,
            0.0,
        )
        cell = grid.world_to_cell(2.0, 0.0)
        self.assertAlmostEqual(
            grid.log_odds[grid._index(*cell)],
            grid.config.miss_log_odds,
        )

    def test_confirmed_track_overrides_same_frame_free_ray(self):
        grid = self.grid()
        grid.update_scan(
            0.0, 0.0, 0.0, [math.inf], 0.0, 0.1,
            0.1, 8.0, 0.0)
        grid.update_obstacles(0.0, 0.0, [(4.0, 0.0)], 0.0)
        self.assertTrue(grid.snapshot(0.0).is_blocked(4.0, 0.0))


if __name__ == '__main__':
    unittest.main()
