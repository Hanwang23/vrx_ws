import math
import unittest

from han_usv_controller.guidance import ILOSPathFollower, plan_dubins_path


class DubinsPathTests(unittest.TestCase):
    def assert_pose_close(self, actual, expected):
        self.assertAlmostEqual(actual[0], expected[0], delta=1e-5)
        self.assertAlmostEqual(actual[1], expected[1], delta=1e-5)
        angle_error = math.atan2(
            math.sin(actual[2] - expected[2]),
            math.cos(actual[2] - expected[2]),
        )
        self.assertAlmostEqual(angle_error, 0.0, delta=1e-5)

    def test_straight_path_has_expected_length_and_endpoint(self):
        path = plan_dubins_path((0.0, 0.0, 0.0), (20.0, 0.0, 0.0), 8.0)
        self.assertAlmostEqual(path.total_length, 20.0, delta=1e-6)
        self.assert_pose_close(path.points[-1], (20.0, 0.0, 0.0))

    def test_arbitrary_paths_reach_goal_pose(self):
        cases = (
            ((0.0, 0.0, 0.0), (24.0, 18.0, math.radians(70.0))),
            ((3.0, -2.0, math.radians(130.0)), (-12.0, 15.0, -1.2)),
            ((0.0, 0.0, math.pi), (2.0, 1.0, 0.1)),
        )
        for start, goal in cases:
            with self.subTest(start=start, goal=goal):
                path = plan_dubins_path(start, goal, 6.0, sample_step=0.4)
                self.assertGreaterEqual(len(path.points), 2)
                self.assert_pose_close(path.points[0], start)
                self.assert_pose_close(path.points[-1], goal)

    def test_sampling_never_jumps_farther_than_step(self):
        path = plan_dubins_path(
            (0.0, 0.0, 0.0),
            (15.0, 12.0, math.pi / 2.0),
            5.0,
            sample_step=0.5,
        )
        distances = [
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(path.points, path.points[1:])
        ]
        self.assertLessEqual(max(distances), 0.501)

    def test_identical_pose_returns_zero_path(self):
        path = plan_dubins_path((2.0, 3.0, 0.4), (2.0, 3.0, 0.4), 5.0)
        self.assertEqual(path.total_length, 0.0)
        self.assertEqual(len(path.points), 1)

    def test_invalid_radius_is_rejected(self):
        with self.assertRaises(ValueError):
            plan_dubins_path((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.0)

    def test_all_six_dubins_families_can_be_shortest(self):
        cases = (
            ('LSL', (1.5, 1.0, math.radians(67.5))),
            ('RSR', (1.5, -1.0, math.radians(-67.5))),
            ('LSR', (1.5, 0.5, 0.0)),
            ('RSL', (1.5, -0.5, 0.0)),
            ('RLR', (1.5, 1.5, math.radians(135.0))),
            ('LRL', (1.5, -1.5, math.radians(-135.0))),
        )
        for expected, goal in cases:
            with self.subTest(expected=expected):
                path = plan_dubins_path((0.0, 0.0, 0.0), goal, 1.0)
                self.assertEqual(''.join(path.modes), expected)

    def test_three_turn_paths_can_be_disabled_for_inertial_vessel(self):
        goal = (1.5, -1.5, math.radians(-135.0))
        unrestricted = plan_dubins_path((0.0, 0.0, 0.0), goal, 1.0)
        vessel_path = plan_dubins_path(
            (0.0, 0.0, 0.0),
            goal,
            1.0,
            allow_three_turn_paths=False,
        )

        self.assertEqual(''.join(unrestricted.modes), 'LRL')
        self.assertNotIn(''.join(vessel_path.modes), ('RLR', 'LRL'))
        self.assert_pose_close(vessel_path.points[-1], goal)

    def test_dubins_samples_include_exact_mode_curvature(self):
        radius = 8.0
        path = plan_dubins_path(
            (0.0, 0.0, 0.0),
            (20.0, 15.0, math.pi / 2.0),
            radius,
            sample_step=0.5,
            allow_three_turn_paths=False,
        )

        self.assertEqual(len(path.curvatures), len(path.points) - 1)
        self.assertTrue(all(
            any(abs(value - expected) < 1e-12 for expected in (
                -1.0 / radius, 0.0, 1.0 / radius))
            for value in path.curvatures
        ))


class ILOSTests(unittest.TestCase):
    def test_straight_path_has_zero_curvature(self):
        follower = ILOSPathFollower(lookahead=8.0)
        follower.reset(((0.0, 0.0), (10.0, 0.0), (20.0, 0.0)))

        output = follower.preview(5.0, 0.0)

        self.assertAlmostEqual(output.path_curvature, 0.0, delta=1e-12)

    def test_sampled_left_arc_reports_inverse_radius_curvature(self):
        radius = 8.0
        points = tuple(
            (
                radius * math.sin(index * 0.1),
                radius * (1.0 - math.cos(index * 0.1)),
            )
            for index in range(16)
        )
        follower = ILOSPathFollower(lookahead=8.0)
        follower.reset(points)

        output = follower.preview(*points[5])

        self.assertAlmostEqual(
            output.path_curvature, 1.0 / radius, delta=0.01)

    def test_follower_uses_supplied_analytic_curvature(self):
        follower = ILOSPathFollower(lookahead=8.0)
        follower.reset(
            ((0.0, 0.0), (1.0, 0.0), (2.0, 0.1)),
            (0.125, 0.125),
        )

        output = follower.preview(0.5, 0.0)

        self.assertAlmostEqual(output.path_curvature, 0.125, delta=1e-12)
        self.assertAlmostEqual(output.upcoming_curvature, 0.125, delta=1e-12)

    def test_curvature_preview_sees_an_arc_before_reaching_it(self):
        follower = ILOSPathFollower(lookahead=8.0)
        follower.reset(
            ((0.0, 0.0), (4.0, 0.0), (8.0, 0.0), (9.0, 0.1)),
            (0.0, 0.0, 0.125),
        )

        output = follower.preview(1.0, 0.0)

        self.assertAlmostEqual(output.path_curvature, 0.0, delta=1e-12)
        self.assertAlmostEqual(output.upcoming_curvature, 0.125, delta=1e-12)

    def test_positive_left_cross_track_commands_right_turn(self):
        follower = ILOSPathFollower(lookahead=8.0)
        follower.reset(((0.0, 0.0), (20.0, 0.0)))
        output = follower.preview(5.0, 4.0)
        self.assertGreater(output.cross_track_error, 0.0)
        self.assertLess(output.course, 0.0)

    def test_integral_bias_strengthens_persistent_correction(self):
        follower = ILOSPathFollower(
            lookahead=8.0,
            integral_gain=0.5,
            integral_limit=3.0,
        )
        follower.reset(((0.0, 0.0), (20.0, 0.0)))
        initial = follower.preview(5.0, 2.0).course
        for _ in range(100):
            follower.integrate(2.0, 0.1, enabled=True)
        corrected = follower.preview(5.0, 2.0).course
        self.assertLess(corrected, initial)
        self.assertLessEqual(abs(follower.integral_bias), 3.0)

    def test_integrator_freezes_during_local_avoidance(self):
        follower = ILOSPathFollower(integral_gain=1.0)
        follower.reset(((0.0, 0.0), (20.0, 0.0)))
        follower.integrate(5.0, 2.0, enabled=False)
        self.assertEqual(follower.integral_bias, 0.0)

    def test_segment_progress_does_not_move_backwards(self):
        follower = ILOSPathFollower()
        follower.reset(((0.0, 0.0), (5.0, 0.0), (10.0, 0.0)))
        follower.preview(8.0, 0.2)
        progressed = follower.segment_index
        follower.preview(1.0, 0.1)
        self.assertGreaterEqual(follower.segment_index, progressed)

    def test_terminal_guidance_turns_back_after_overshoot(self):
        follower = ILOSPathFollower()
        follower.reset(((0.0, 0.0), (5.0, 0.0), (10.0, 0.0)))
        output = follower.preview(12.0, 0.0)
        self.assertAlmostEqual(abs(output.course), math.pi, delta=1e-6)

    def test_partial_horizon_guidance_continues_forward_after_endpoint(self):
        follower = ILOSPathFollower()
        follower.reset(((0.0, 0.0), (5.0, 0.0), (10.0, 0.0)))

        output = follower.preview(
            12.0, 0.0, return_to_endpoint=False)

        self.assertAlmostEqual(output.course, 0.0, delta=1e-6)

    def test_nearest_search_cannot_skip_far_ahead_on_self_near_path(self):
        follower = ILOSPathFollower()
        points = [(float(index), 0.0) for index in range(12)]
        points += [(0.1, 0.1), (0.2, 0.1), (20.0, 0.0)]
        follower.reset(points)
        follower.preview(0.1, 0.1)
        self.assertLessEqual(follower.segment_index, 10)


if __name__ == '__main__':
    unittest.main()
