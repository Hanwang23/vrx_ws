import math
import unittest

from codex_usv_controller.occupancy_grid import OccupancySnapshot
from codex_usv_controller.state_lattice import (
    DubinsStateLatticePlanner,
    StateLatticeConfig,
)


def make_grid(blocked_cells=(), width=80, height=60, resolution=1.0):
    blocked = [False] * (width * height)
    for column, row in blocked_cells:
        blocked[row * width + column] = True
    return OccupancySnapshot(
        origin_east=-10.0,
        origin_north=-30.0,
        resolution=resolution,
        width=width,
        height=height,
        probabilities=tuple(-1 for _ in blocked),
        blocked=tuple(blocked),
        revision=7,
    )


class DubinsStateLatticeTests(unittest.TestCase):
    def planner(self, **overrides):
        values = dict(
            turn_radius=4.0,
            sample_step=0.5,
            heading_bins=16,
            planning_horizon=30.0,
            analytic_expansion_distance=8.0,
            max_expansions=4000,
        )
        values.update(overrides)
        return DubinsStateLatticePlanner(StateLatticeConfig(**values))

    def test_empty_map_uses_collision_checked_analytic_path(self):
        grid = make_grid()
        plan = self.planner().plan((0.0, 0.0, 0.0), (20.0, 0.0, 0.0), grid)
        self.assertIsNotNone(plan)
        self.assertTrue(plan.reached_goal)
        self.assertFalse(plan.used_search)
        self.assertAlmostEqual(plan.path.points[-1][0], 20.0, places=6)

    def test_wall_forces_lattice_search_around_endpoint(self):
        wall = [(20, row) for row in range(20, 41)]
        grid = make_grid(wall)
        plan = self.planner().plan((0.0, 0.0, 0.0), (20.0, 0.0, 0.0), grid)
        self.assertIsNotNone(plan)
        self.assertTrue(plan.used_search)
        self.assertGreater(plan.expanded_states, 0)
        self.assertTrue(all(
            not grid.is_blocked(point[0], point[1])
            for point in plan.path.points))

    def test_blocked_start_cell_can_escape_clearance_bubble(self):
        grid = make_grid([(10, 30)])
        plan = self.planner().plan((0.0, 0.0, 0.0), (20.0, 0.0, 0.0), grid)
        self.assertIsNotNone(plan)
        self.assertAlmostEqual(plan.path.points[-1][0], 20.0, places=6)

    def test_obstacle_beyond_start_clearance_still_blocks_direct_path(self):
        grid = make_grid([(15, 30)])
        plan = self.planner().plan((0.0, 0.0, 0.0), (20.0, 0.0, 0.0), grid)
        self.assertIsNotNone(plan)
        self.assertTrue(plan.used_search)

    def test_real_obstacle_inside_clearance_blocks_when_start_is_free(self):
        grid = make_grid([(13, 30)])
        planner = self.planner()
        self.assertFalse(planner._collision_free(
            ((1.0, 0.0, 0.0), (3.0, 0.0, 0.0)),
            grid,
            clearance_center=(0.0, 0.0, 0.0),
        ))

    def test_start_clearance_cannot_be_reentered_after_exit(self):
        grid = make_grid([(10, 30), (12, 30)])
        planner = self.planner()
        self.assertFalse(planner._collision_free(
            (
                (0.0, 0.0, 0.0),
                (5.0, 0.0, 0.0),
                (2.0, 0.0, math.pi),
            ),
            grid,
            clearance_center=(0.0, 0.0, 0.0),
        ))

    def test_far_goal_is_clipped_to_rolling_horizon(self):
        plan = self.planner(planning_horizon=25.0).plan(
            (0.0, 0.0, 0.0), (60.0, 0.0, 0.0), make_grid())
        self.assertIsNotNone(plan)
        self.assertFalse(plan.reached_goal)
        self.assertAlmostEqual(plan.path.points[-1][0], 25.0, places=6)

    def test_all_samples_respect_curvature_limit(self):
        planner = self.planner()
        plan = planner.plan(
            (0.0, 0.0, 0.0), (16.0, 12.0, math.pi / 2.0), make_grid())
        self.assertTrue(all(
            abs(curvature) <= 1.0 / 4.0 + 1e-12
            for curvature in plan.path.curvatures))


if __name__ == '__main__':
    unittest.main()
