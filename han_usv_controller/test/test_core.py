import math
import unittest
from unittest.mock import patch

from han_usv_controller.core import (
    ControlConfig,
    ControllerCore,
    GeoTarget,
    GroundSpeedEstimator,
    PIDController,
    ReactiveAvoidance,
    VesselState,
    distance_and_bearing,
    extract_obstacle_points,
    nearest_neighbor_order,
    normalize_angle,
    validate_control_config,
)
from han_usv_controller.occupancy_grid import OccupancySnapshot


class CoreMathTest(unittest.TestCase):
    def test_angle_wrap(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), -math.pi)

    def test_geodetic_bearing(self):
        distance, bearing = distance_and_bearing(0.0, 0.0, 0.001, 0.0)
        self.assertAlmostEqual(distance, 111.19, delta=0.2)
        self.assertAlmostEqual(bearing, math.pi / 2.0, places=6)

    def test_pid_anti_windup_and_limit(self):
        pid = PIDController(10.0, 10.0, 0.0, 5.0, 100.0)
        for _ in range(100):
            self.assertEqual(pid.update(10.0, 0.1), 5.0)
        self.assertAlmostEqual(pid.integral, 0.0)

    def test_pid_respects_asymmetric_output_headroom(self):
        pid = PIDController(10.0, 10.0, 0.0, 100.0, 100.0)
        for _ in range(20):
            self.assertEqual(
                pid.update(10.0, 0.1, output_limits=(-20.0, 5.0)),
                5.0,
            )
        self.assertAlmostEqual(pid.integral, 0.0)

    def test_speed_estimator_keeps_velocity_direction(self):
        estimator = GroundSpeedEstimator(smoothing=1.0)
        estimator.update(0.0, 0.0, 0.0)
        estimator.update(0.0, 0.00001, 1.0)
        self.assertGreater(estimator.velocity_east, 1.0)
        self.assertAlmostEqual(estimator.velocity_north, 0.0, places=6)

    def test_nearest_neighbor_order(self):
        near = GeoTarget(0.0, 0.001)
        far = GeoTarget(0.0, 0.01)
        ordered = nearest_neighbor_order(0.0, 0.0, [far, near])
        self.assertEqual(ordered, [near, far])

    def test_invalid_control_config_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_control_config(ControlConfig(
                position_tolerance=5.0,
                waypoint_exit_tolerance=4.0,
            ))
        with self.assertRaises(ValueError):
            validate_control_config(ControlConfig(
                obstacle_emergency_distance=22.0,
                obstacle_warning_distance=18.0,
            ))
        with self.assertRaises(ValueError):
            validate_control_config(ControlConfig(
                max_turn_thrust=100.0,
                max_low_speed_turn_thrust=120.0,
            ))
        with self.assertRaises(ValueError):
            validate_control_config(ControlConfig(
                speed_brake_deadband=-0.1,
            ))
        with self.assertRaises(ValueError):
            validate_control_config(ControlConfig(
                guidance_replan_cooldown=-0.1,
            ))
        with self.assertRaises(ValueError):
            validate_control_config(ControlConfig(
                obstacle_clear_hold_time=0.0,
            ))
        with self.assertRaises(ValueError):
            validate_control_config(ControlConfig(
                max_alignment_yaw_acceleration=0.0,
            ))
        with self.assertRaises(ValueError):
            validate_control_config(ControlConfig(
                alignment_heading_rate_gain=-0.1,
            ))
        with self.assertRaises(ValueError):
            validate_control_config(ControlConfig(
                max_alignment_yaw_acceleration=math.nan,
            ))
        with self.assertRaises(ValueError):
            validate_control_config(ControlConfig(
                max_alignment_yaw_rate=math.inf,
            ))


class AvoidanceTest(unittest.TestCase):
    def test_clear_sector_tracks_target_relative_heading(self):
        target_angle = math.radians(35.0)
        decision = ReactiveAvoidance().compute(
            (), 0.0, 0.0, ControlConfig(), target_angle=target_angle)
        self.assertAlmostEqual(decision.steering_angle, target_angle)
        self.assertEqual(decision.speed_scale, 1.0)
        self.assertTrue(math.isinf(decision.nearest_obstacle))

    def test_center_obstacle_stops_and_turns_toward_clear_side(self):
        config = ControlConfig(
            obstacle_warning_distance=15.0,
            obstacle_emergency_distance=5.0,
        )
        ranges = [30.0] * 181
        angle_min = -math.pi / 2.0
        increment = math.pi / 180.0
        # Obstacle is ahead-left, so the clear escape side is right.
        for index in range(92, 111):
            ranges[index] = 4.0

        decision = ReactiveAvoidance().compute(
            ranges, angle_min, increment, config)
        self.assertLess(decision.steering_angle, 0.0)
        self.assertAlmostEqual(decision.nearest_obstacle, 4.0)
        self.assertAlmostEqual(decision.collision_clearance, 3.8, delta=0.2)

    def test_wide_front_sector_and_close_valid_returns_trigger(self):
        config = ControlConfig(
            obstacle_warning_distance=15.0,
            obstacle_emergency_distance=5.0,
            obstacle_front_angle=math.radians(65.0),
        )
        ranges = [30.0] * 181
        angle_min = -math.pi / 2.0
        increment = math.pi / 180.0
        ranges[139] = 0.3
        ranges[140] = 0.3
        decision = ReactiveAvoidance().compute(
            ranges, angle_min, increment, config, range_min=0.1)
        self.assertNotEqual(decision.steering_angle, 0.0)
        self.assertAlmostEqual(decision.nearest_obstacle, 0.3)
        self.assertLess(decision.collision_clearance, 0.4)

    def test_negative_infinite_return_is_treated_as_too_close(self):
        ranges = [math.inf] * 181
        ranges[90] = -math.inf
        decision = ReactiveAvoidance().compute(
            ranges,
            -math.pi / 2.0,
            math.pi / 180.0,
            ControlConfig(),
            range_min=0.1,
        )
        self.assertAlmostEqual(0.1, decision.nearest_obstacle)
        self.assertLess(decision.speed_scale, 0.1)

    def test_point_cloud_obstacle_supplements_empty_scan(self):
        config = ControlConfig(
            obstacle_warning_distance=15.0,
            obstacle_emergency_distance=5.0,
        )
        points = [(4.0, math.radians(10.0))] * 3
        decision = ReactiveAvoidance().compute(
            (), 0.0, 0.0, config, obstacle_points=points)
        self.assertNotEqual(decision.steering_angle, 0.0)
        self.assertAlmostEqual(decision.nearest_obstacle, 4.0)
        self.assertAlmostEqual(decision.collision_clearance, 3.94, delta=0.1)

    def test_wide_gate_keeps_a_nearly_straight_course(self):
        config = ControlConfig()
        points = (
            [(10.0, math.radians(-45.0))] * 3
            + [(10.0, math.radians(45.0))] * 3
        )
        decision = ReactiveAvoidance().compute(
            (), 0.0, 0.0, config, obstacle_points=points)
        self.assertLess(abs(decision.steering_angle), math.radians(4.0))
        self.assertGreater(decision.speed_scale, 0.4)

    def test_gap_direction_does_not_flip_on_small_scan_jitter(self):
        config = ControlConfig()
        avoidance = ReactiveAvoidance()
        first = avoidance.compute(
            (), 0.0, 0.0, config,
            obstacle_points=[(10.0, 0.0)] * 3,
        )
        second = avoidance.compute(
            (), 0.0, 0.0, config,
            obstacle_points=[(10.0, math.radians(-2.0))] * 3,
        )
        self.assertGreater(first.steering_angle * second.steering_angle, 0.0)

    def test_target_behind_keeps_one_turn_direction_across_angle_wrap(self):
        avoidance = ReactiveAvoidance()
        first = avoidance.compute(
            (), 0.0, 0.0, ControlConfig(),
            target_angle=math.radians(179.0),
        )
        second = avoidance.compute(
            (), 0.0, 0.0, ControlConfig(),
            target_angle=math.radians(-179.0),
        )
        self.assertGreater(first.steering_angle, 0.0)
        self.assertGreater(second.steering_angle, 0.0)

    def test_side_obstacle_does_not_stop_a_clear_selected_path(self):
        config = ControlConfig()
        decision = ReactiveAvoidance().compute(
            (), 0.0, 0.0, config,
            obstacle_points=[(7.0, math.radians(60.0))] * 3,
        )
        self.assertGreater(decision.speed_scale, 0.9)
        self.assertTrue(math.isinf(decision.path_clearance))

    def test_single_scan_ray_is_rejected_as_noise(self):
        ranges = [30.0] * 181
        ranges[90] = 0.3
        decision = ReactiveAvoidance().compute(
            ranges,
            -math.pi / 2.0,
            math.pi / 180.0,
            ControlConfig(),
        )
        self.assertTrue(math.isinf(decision.nearest_obstacle))

    def test_clearance_uses_nearest_supported_obstacle(self):
        decision = ReactiveAvoidance().compute(
            (), 0.0, 0.0, ControlConfig(),
            obstacle_points=(
                (0.5, 0.0),
                (16.0, math.radians(20.0)),
            ),
        )
        self.assertAlmostEqual(decision.nearest_obstacle, 0.5)
        self.assertLess(decision.collision_clearance, 0.6)


class PointCloudFilterTest(unittest.TestCase):
    def test_low_buoy_survives_water_plane_filter(self):
        water = [
            (float(x), float(y), -1.80 + 0.01 * x - 0.005 * y)
            for x in range(2, 13)
            for y in range(-5, 6)
        ]
        buoy = [
            (6.0 + 0.05 * x, 0.05 * y, -1.58 + 0.03 * z)
            for x in range(-2, 3)
            for y in range(-2, 3)
            for z in range(3)
        ]
        obstacles = extract_obstacle_points(
            water + buoy, 18.0, math.radians(100.0))
        self.assertEqual(len(obstacles), 1)
        self.assertAlmostEqual(obstacles[0][0], 6.0, delta=0.5)

    def test_isolated_point_above_water_is_rejected(self):
        water = [
            (float(x), float(y), -1.8)
            for x in range(2, 13)
            for y in range(-5, 6)
        ]
        obstacles = extract_obstacle_points(
            water + [(5.0, 0.0, -1.5)],
            18.0,
            math.radians(100.0),
        )
        self.assertEqual(obstacles, [])

    def test_tilted_water_plane_does_not_reapply_absolute_height_cutoff(self):
        water = [
            (float(x), 0.2 * y, -2.30 + 0.10 * x)
            for x in range(2, 13)
            for y in range(-5, 6)
        ]
        low_buoy = [
            (2.4 + 0.04 * x, 0.04 * y, -1.81)
            for x in range(-1, 2)
            for y in range(-1, 2)
        ]
        obstacles = extract_obstacle_points(
            water + low_buoy,
            18.0,
            math.radians(100.0),
            minimum_height=-1.75,
        )
        self.assertEqual(len(obstacles), 1)
        self.assertAlmostEqual(obstacles[0][0], 2.4, delta=0.5)


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.config = ControlConfig(
            obstacle_avoidance_enabled=False,
            waypoint_dwell_time=0.1,
        )
        self.controller = ControllerCore(self.config)

    def test_forward_mixing_for_target_ahead(self):
        self.controller.set_targets([GeoTarget(0.0, 0.001, 0.0)], 'wayfinding')
        command = self.controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)
        self.assertEqual(command.state, 'navigating')
        self.assertGreater(command.left_thrust, 0.0)
        self.assertAlmostEqual(command.left_thrust, command.right_thrust)

    def test_positive_heading_error_matches_manual_left_turn(self):
        self.controller.set_targets([GeoTarget(0.001, 0.0, math.pi / 2)], 'wayfinding')
        command = self.controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)
        self.assertLess(command.left_thrust, command.right_thrust)

    def test_colregs_give_way_bias_reaches_speed_and_heading_control(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=False,
        ))
        controller.set_targets([GeoTarget(0.0, 0.001)], 'wayfinding')
        command = controller.update(VesselState(
            0.0,
            0.0,
            0.0,
            0.5,
            colregs_active=True,
            colregs_heading_bias=math.radians(-20.0),
            colregs_speed_scale=0.5,
            colregs_action='give_way_starboard',
        ), 0.05)

        self.assertEqual('colregs_give_way', command.state)
        self.assertEqual('give_way_starboard', command.colregs_action)
        self.assertLess(command.heading_error, 0.0)
        self.assertLessEqual(command.desired_speed, 0.8)
        self.assertGreater(command.left_thrust, command.right_thrust)

    def test_dubins_ilos_guidance_is_planned_for_oriented_target(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=True,
        ))
        controller.set_targets(
            [GeoTarget(0.001, 0.0, math.pi / 2.0)], 'wayfinding')

        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)

        self.assertEqual(command.guidance_mode, 'dubins_ilos')
        self.assertTrue(command.path_valid)
        self.assertEqual(command.path_revision, 1)
        self.assertGreater(len(command.path_points_body), 2)
        self.assertTrue(math.isfinite(command.path_remaining))

    def test_dubins_curvature_adds_yaw_rate_feedforward(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=True,
            curvature_feedforward_gain=1.0,
        ))
        controller.set_targets(
            [GeoTarget(0.001, 0.0, math.pi / 2.0)], 'wayfinding')

        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 1.0), 0.05)

        self.assertGreater(command.path_curvature, 0.0)
        self.assertGreater(command.yaw_rate_feedforward, 0.0)
        self.assertAlmostEqual(
            command.yaw_rate_feedforward,
            command.desired_speed * command.path_curvature,
            delta=1e-9,
        )

    def test_obstacle_beyond_nearby_goal_is_outside_planning_horizon(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
        ))
        target_distance = 5.0
        controller.set_targets(
            [GeoTarget(0.0, target_distance / 111194.9, None)],
            'wayfinding',
        )

        command = controller.update(VesselState(
            0.0,
            0.0,
            0.0,
            0.5,
            obstacle_points=((15.0, 0.0),),
        ), 0.05)

        self.assertEqual(command.state, 'navigating')
        self.assertFalse(command.avoidance_override)
        self.assertTrue(math.isinf(command.nearest_obstacle))

    def test_terminal_capture_region_cannot_trigger_stuck_backup(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=True,
            terminal_recovery_disable_radius=8.0,
        )
        controller = ControllerCore(config)
        controller.set_targets(
            [GeoTarget(0.0, 5.0 / 111194.9, None)], 'wayfinding')
        controller.update(VesselState(0.0, 0.0, 0.0, 0.0), 0.05)
        controller.no_progress_elapsed = config.obstacle_stuck_timeout + 1.0

        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)

        self.assertNotEqual(command.state, 'backing_away')
        self.assertEqual(controller.backup_remaining, 0.0)

    def test_target_without_yaw_uses_ilos_line(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=True,
        ))
        controller.set_targets([GeoTarget(0.0, 0.001, None)], 'wayfinding')

        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)

        self.assertEqual(command.guidance_mode, 'ilos_line')
        self.assertTrue(command.path_valid)

    def test_ilos_integrator_freezes_while_avoidance_overrides_path(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.001, 0.0)], 'wayfinding')
        controller.update(VesselState(
            0.0, 0.0, 0.0, 1.0,
        ), 0.05)
        controller.ilos.integral_bias = 0.75

        command = controller.update(VesselState(
            0.00001, 0.0, 0.0, 1.0,
            obstacle_points=((10.0, 0.0),),
        ), 0.2)

        self.assertTrue(command.avoidance_override)
        self.assertAlmostEqual(controller.ilos.integral_bias, 0.75)

    def test_large_deviation_after_avoidance_triggers_one_replan(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
            guidance_replan_path_deviation=8.0,
            guidance_replan_cooldown=5.0,
            obstacle_clear_hold_time=0.5,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.001, 0.0)], 'wayfinding')

        avoiding = controller.update(VesselState(
            0.0, 0.0, 0.0, 1.0,
            obstacle_points=((10.0, 0.0),),
        ), 0.05)
        first_clear = controller.update(VesselState(
            10.0 / 111194.9, 0.0, 0.0, 1.0,
        ), 0.25)
        cleared = controller.update(VesselState(
            10.0 / 111194.9, 0.0, 0.0, 1.0,
        ), 0.25)
        controller.ilos.integral_bias = 0.75
        committed = controller.update(VesselState(
            10.0 / 111194.9, 0.0, 0.0, 1.0,
        ), 0.05)
        replanned = controller.update(VesselState(
            10.0 / 111194.9, 0.0, 0.0, 1.0,
        ), 0.05)
        settled = controller.update(VesselState(
            10.0 / 111194.9, 0.0, 0.0, 1.0,
        ), 0.05)

        self.assertTrue(avoiding.avoidance_override)
        self.assertTrue(first_clear.avoidance_episode_active)
        self.assertFalse(first_clear.guidance_replan_pending)
        self.assertTrue(cleared.guidance_replan_pending)
        self.assertEqual(cleared.path_revision, 1)
        self.assertFalse(committed.guidance_replanned)
        self.assertTrue(committed.guidance_replan_pending)
        self.assertEqual(committed.path_revision, 1)
        self.assertEqual(committed.ilos_integral_bias, 0.75)
        self.assertTrue(replanned.guidance_replanned)
        self.assertFalse(replanned.guidance_replan_pending)
        self.assertEqual(replanned.path_revision, 2)
        self.assertEqual('path_deviation', replanned.guidance_replan_reason)
        self.assertEqual(replanned.ilos_integral_bias, 0.0)
        self.assertAlmostEqual(replanned.path_deviation, 0.0, places=6)
        self.assertAlmostEqual(replanned.path_points_body[0][0], 0.0, places=6)
        self.assertAlmostEqual(replanned.path_points_body[0][1], 0.0, places=6)
        self.assertEqual('', settled.guidance_replan_reason)

    def test_small_post_avoidance_error_keeps_existing_path(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.001, 0.0)], 'wayfinding')
        controller.update(VesselState(
            0.0, 0.0, 0.0, 1.0,
            obstacle_points=((10.0, 0.0),),
        ), 0.05)

        command = None
        for _ in range(2):
            command = controller.update(VesselState(
                1.0 / 111194.9, 0.0, 0.0, 1.0,
            ), 0.25)

        assert command is not None
        self.assertFalse(command.guidance_replan_pending)
        self.assertEqual(command.path_revision, 1)

    def test_single_clear_frame_does_not_release_avoidance_episode(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
            obstacle_clear_hold_time=0.5,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.001, 0.0)], 'wayfinding')
        controller.update(VesselState(
            0.0, 0.0, 0.0, 1.0,
            obstacle_points=((10.0, 0.0),),
        ), 0.05)

        clear = controller.update(VesselState(
            10.0 / 111194.9, 0.0, 0.0, 1.0,
        ), 0.25)
        obstacle_returns = controller.update(VesselState(
            10.0 / 111194.9, 0.0, 0.0, 1.0,
            obstacle_points=((4.0, 0.0),),
        ), 0.05)

        self.assertTrue(clear.avoidance_episode_active)
        self.assertFalse(clear.guidance_replan_pending)
        self.assertTrue(obstacle_returns.avoidance_episode_active)
        self.assertFalse(obstacle_returns.guidance_replan_pending)

    def test_obstacle_return_cancels_pending_without_starting_cooldown(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
            obstacle_clear_hold_time=0.5,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.001, 0.0)], 'wayfinding')
        controller.update(VesselState(
            0.0, 0.0, 0.0, 1.0,
            obstacle_points=((10.0, 0.0),),
        ), 0.05)
        pending = None
        for _ in range(2):
            pending = controller.update(VesselState(
                10.0 / 111194.9, 0.0, 0.0, 1.0,
            ), 0.25)

        obstacle_returns = controller.update(VesselState(
            10.0 / 111194.9, 0.0, 0.0, 1.0,
            obstacle_points=((4.0, 0.0),),
        ), 0.05)

        assert pending is not None
        self.assertTrue(pending.guidance_replan_pending)
        self.assertFalse(obstacle_returns.guidance_replan_pending)
        self.assertFalse(obstacle_returns.guidance_replanned)
        self.assertTrue(obstacle_returns.avoidance_episode_active)
        self.assertEqual(obstacle_returns.path_revision, 1)
        self.assertEqual(
            obstacle_returns.guidance_replan_cooldown_remaining, 0.0)

    def test_large_deviation_without_avoidance_does_not_replan(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.001, 0.0)], 'wayfinding')
        controller.update(VesselState(
            0.0, 0.0, 0.0, 1.0,
        ), 0.05)

        command = controller.update(VesselState(
            10.0 / 111194.9, 0.0, 0.0, 1.0,
        ), 1.0)

        self.assertGreater(command.path_deviation, 8.0)
        self.assertFalse(command.guidance_replan_pending)
        self.assertEqual(command.path_revision, 1)

    def test_ilos_line_does_not_trigger_dubins_replan(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
            obstacle_clear_hold_time=0.5,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.001, None)], 'wayfinding')
        controller.update(VesselState(
            0.0, 0.0, 0.0, 1.0,
            obstacle_points=((10.0, 0.0),),
        ), 0.05)

        command = None
        for _ in range(2):
            command = controller.update(VesselState(
                10.0 / 111194.9, 0.0, 0.0, 1.0,
            ), 0.25)

        assert command is not None
        self.assertEqual(command.guidance_mode, 'ilos_line')
        self.assertFalse(command.guidance_replan_pending)
        self.assertEqual(command.path_revision, 1)

    def test_emergency_latch_release_can_trigger_replan(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
            obstacle_clear_hold_time=0.5,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.001, 0.0)], 'wayfinding')
        braking = controller.update(VesselState(
            0.0, 0.0, 0.0, 1.0,
            obstacle_points=((4.0, 0.0),),
        ), 0.05)

        command = None
        for _ in range(3):
            command = controller.update(VesselState(
                10.0 / 111194.9, 0.0, 0.0, 0.0,
            ), 0.25)

        assert command is not None
        self.assertEqual(braking.state, 'braking')
        self.assertTrue(command.guidance_replan_pending)

    def test_replan_cooldown_blocks_repeated_path_churn(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
            guidance_replan_cooldown=5.0,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.001, 0.0)], 'wayfinding')
        controller.update(VesselState(
            0.0, 0.0, 0.0, 1.0,
            obstacle_points=((10.0, 0.0),),
        ), 0.05)
        for _ in range(2):
            controller.update(VesselState(
                10.0 / 111194.9, 0.0, 0.0, 1.0,
            ), 0.25)
        controller.update(VesselState(
            10.0 / 111194.9, 0.0, 0.0, 1.0,
        ), 0.05)
        controller.update(VesselState(
            10.0 / 111194.9, 0.0, 0.0, 1.0,
            obstacle_points=((10.0, 0.0),),
        ), 0.05)

        command = None
        for _ in range(2):
            command = controller.update(VesselState(
                20.0 / 111194.9, 0.0, 0.0, 1.0,
            ), 0.25)

        assert command is not None
        self.assertFalse(command.guidance_replan_pending)
        self.assertEqual(command.path_revision, 2)

    def test_terminal_region_suppresses_post_avoidance_replan(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
            obstacle_clear_hold_time=0.5,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 50.0 / 111194.9, 0.0)], 'wayfinding')
        controller.update(VesselState(
            0.0, 0.0, 0.0, 0.5,
            obstacle_points=((10.0, 0.0),),
        ), 0.05)

        command = None
        for _ in range(2):
            command = controller.update(VesselState(
                10.0 / 111194.9,
                45.0 / 111194.9,
                math.pi,
                0.5,
            ), 0.25)

        assert command is not None
        self.assertAlmostEqual(command.path_deviation, 0.0, delta=1e-6)
        self.assertLess(command.distance, 2.0 * controller.config.dubins_turn_radius)
        self.assertFalse(command.guidance_replan_pending)
        self.assertFalse(command.guidance_replanned)
        self.assertEqual(command.guidance_replan_reason, '')
        self.assertEqual(command.guidance_mode, 'ilos_line')
        self.assertEqual(command.path_revision, 2)

    def test_pending_replan_is_cancelled_after_entering_terminal_region(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
            obstacle_clear_hold_time=0.5,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 50.0 / 111194.9, 0.0)], 'wayfinding')
        controller.update(VesselState(
            0.0, 0.0, 0.0, 0.5,
            obstacle_points=((10.0, 0.0),),
        ), 0.05)
        pending = None
        for _ in range(2):
            pending = controller.update(VesselState(
                10.0 / 111194.9, 0.0, 0.0, 0.5,
            ), 0.25)

        terminal = controller.update(VesselState(
            10.0 / 111194.9,
            45.0 / 111194.9,
            0.0,
            0.5,
        ), 0.05)

        assert pending is not None
        self.assertTrue(pending.guidance_replan_pending)
        self.assertFalse(terminal.guidance_replan_pending)
        self.assertTrue(terminal.guidance_replanned)
        self.assertEqual(terminal.guidance_replan_reason, 'terminal_approach')
        self.assertEqual(terminal.guidance_mode, 'ilos_line')
        self.assertEqual(terminal.path_revision, 2)

    def test_lattice_path_switches_to_terminal_line_only_once(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=True,
            lattice_enabled=True,
        ))
        target_longitude = math.degrees(50.0 / 6371000.0)
        near_longitude = math.degrees(40.0 / 6371000.0)
        controller.set_targets(
            [GeoTarget(0.0, target_longitude, 0.0)], 'wayfinding')
        width = height = 120
        grid = OccupancySnapshot(
            -60.0, -60.0, 1.0, width, height,
            tuple(-1 for _ in range(width * height)),
            tuple(False for _ in range(width * height)),
            1,
        )

        initial = controller.update(VesselState(
            0.0, 0.0, 0.0, 0.5,
            east=0.0, north=0.0, occupancy_grid=grid), 0.05)
        terminal = controller.update(VesselState(
            0.0, near_longitude, 0.0, 0.5,
            east=40.0, north=0.0, occupancy_grid=grid), 0.05)
        repeated = controller.update(VesselState(
            0.0, near_longitude, 0.0, 0.5,
            east=40.0, north=0.0, occupancy_grid=grid), 0.05)

        self.assertEqual(initial.guidance_mode, 'lattice_ilos')
        self.assertEqual(terminal.guidance_mode, 'ilos_line')
        self.assertTrue(terminal.guidance_replanned)
        self.assertEqual(terminal.guidance_replan_reason, 'terminal_approach')
        self.assertEqual(terminal.path_revision, initial.path_revision + 1)
        self.assertEqual(terminal.lattice_map_revision, -1)
        self.assertFalse(repeated.guidance_replanned)
        self.assertEqual(repeated.path_revision, terminal.path_revision)

    def test_terminal_line_uses_hysteresis_before_restoring_lattice_path(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=True,
            lattice_enabled=True,
        ))
        target_longitude = math.degrees(50.0 / 6371000.0)
        controller.set_targets(
            [GeoTarget(0.0, target_longitude, 0.0)], 'wayfinding')
        width = height = 120
        grid = OccupancySnapshot(
            -60.0, -60.0, 1.0, width, height,
            tuple(-1 for _ in range(width * height)),
            tuple(False for _ in range(width * height)),
            1,
        )

        initial = controller.update(VesselState(
            0.0, 0.0, 0.0, 0.5,
            east=0.0, north=0.0, occupancy_grid=grid), 0.05)
        terminal = controller.update(VesselState(
            0.0, math.degrees(40.0 / 6371000.0), 0.0, 0.5,
            east=40.0, north=0.0, occupancy_grid=grid), 0.05)
        modest_drift = controller.update(VesselState(
            0.0, math.degrees(32.0 / 6371000.0), 0.0, 0.5,
            east=32.0, north=0.0, occupancy_grid=grid), 0.05)
        recovered = controller.update(VesselState(
            0.0, math.degrees(24.0 / 6371000.0), 0.0, 0.5,
            east=24.0, north=0.0, occupancy_grid=grid), 0.05)

        self.assertEqual(initial.guidance_mode, 'lattice_ilos')
        self.assertEqual(terminal.guidance_mode, 'ilos_line')
        self.assertEqual(modest_drift.guidance_mode, 'ilos_line')
        self.assertFalse(modest_drift.guidance_replanned)
        self.assertEqual(recovered.guidance_mode, 'lattice_ilos')
        self.assertTrue(recovered.guidance_replanned)
        self.assertEqual(
            recovered.guidance_replan_reason, 'terminal_recovery')
        self.assertLess(recovered.path_remaining, 35.0)

    def test_non_navigation_state_cancels_replan_episode(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            guidance_enabled=True,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.0, None)], 'wayfinding')
        controller.guidance_avoidance_episode_active = True
        controller.guidance_avoidance_clear_elapsed = 0.25
        controller.guidance_replan_pending = True
        controller.guidance_replan_pending_reason = 'path_deviation'

        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)

        self.assertEqual(command.state, 'waypoint_dwell')
        self.assertFalse(controller.guidance_avoidance_episode_active)
        self.assertFalse(controller.guidance_replan_pending)

    def test_temporary_stop_preserves_path_replan_cooldown(self):
        controller = ControllerCore(ControlConfig())
        controller.guidance_avoidance_episode_active = True
        controller.guidance_replan_pending = True
        controller.guidance_replan_pending_reason = 'path_deviation'
        controller.guidance_replan_activation_pending = True
        controller.guidance_replan_activation_reason = 'path_deviation'
        controller.guidance_replan_cooldown_remaining = 3.0
        controller.path_revision = 2

        command = controller.stop('lidar_timeout')

        self.assertFalse(controller.guidance_avoidance_episode_active)
        self.assertFalse(controller.guidance_replan_pending)
        self.assertTrue(controller.guidance_replan_activation_pending)
        self.assertEqual(controller.guidance_replan_cooldown_remaining, 3.0)
        self.assertEqual(command.path_revision, 2)
        self.assertTrue(command.guidance_replan_pending)
        self.assertEqual(command.guidance_replan_cooldown_remaining, 3.0)

    def test_alignment_discards_staged_path_without_revision_increment(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=True,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.001, 0.0)], 'wayfinding')
        initial = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.5), 0.05)
        controller.guidance_replan_activation_pending = True
        controller.guidance_replan_activation_reason = 'path_deviation'

        controller._begin_alignment()

        self.assertEqual(initial.path_revision, 1)
        self.assertEqual(controller.path_revision, 1)
        self.assertFalse(controller.guidance_replan_activation_pending)
        self.assertIsNone(controller.guidance_path)
        resumed = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.5), 0.05)
        self.assertEqual(resumed.path_revision, 2)
        self.assertFalse(resumed.guidance_replanned)

    def test_new_targets_discard_staged_path_without_phantom_revision(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=True,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.001, 0.0)], 'wayfinding')
        controller.update(VesselState(0.0, 0.0, 0.0, 0.5), 0.05)
        controller.guidance_replan_activation_pending = True
        controller.guidance_replan_activation_reason = 'path_deviation'

        controller.set_targets(
            [GeoTarget(0.001, 0.0, math.pi / 2.0)], 'wayfinding')
        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.5), 0.05)

        self.assertEqual(command.path_revision, 2)
        self.assertFalse(command.guidance_replanned)

    def test_navigation_caps_requested_yaw_rate(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            max_turn_thrust=900.0,
            max_low_speed_turn_thrust=900.0,
            navigation_heading_rate_gain=0.55,
            navigation_yaw_rate_gain=1000.0,
            max_navigation_yaw_rate=math.radians(15.0),
            max_navigation_yaw_acceleration=math.radians(1000.0),
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.001, 0.0)], 'wayfinding')

        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0, yaw_rate=0.0), 0.05)

        expected_turn = 1000.0 * math.radians(15.0)
        self.assertAlmostEqual(command.left_thrust, -expected_turn)
        self.assertAlmostEqual(command.right_thrust, expected_turn)

    def test_normal_approach_uses_limited_reverse_thrust_to_remove_inertia(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=False,
            max_normal_brake_thrust=160.0,
            speed_brake_deadband=0.2,
        )
        controller = ControllerCore(config)
        controller.set_targets(
            [GeoTarget(0.0, 4.0 / 111194.9, None)], 'wayfinding')

        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 1.5), 0.05)

        self.assertEqual(command.state, 'approach_braking')
        self.assertLess(command.left_thrust + command.right_thrust, 0.0)
        self.assertGreaterEqual(
            command.left_thrust + command.right_thrust,
            -2.0 * config.max_normal_brake_thrust,
        )

    def test_far_route_coasts_instead_of_repeatedly_reverse_braking(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=False,
            normal_brake_distance=15.0,
        )
        controller = ControllerCore(config)
        controller.set_targets(
            [GeoTarget(0.0, 100.0 / 111194.9, None)], 'wayfinding')

        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 2.2), 0.05)

        self.assertNotEqual(command.state, 'approach_braking')
        self.assertGreaterEqual(command.left_thrust + command.right_thrust, 0.0)

    def test_short_line_leg_slows_while_turning_into_the_path(self):
        bearing = math.radians(55.0)
        distance = 14.0
        north = distance * math.sin(bearing)
        east = distance * math.cos(bearing)
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=True,
            lattice_enabled=False,
        ))
        controller.set_targets([
            GeoTarget(
                north / 111194.9,
                east / 111194.9,
                bearing,
            ),
        ], 'wayfinding')

        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)

        self.assertEqual(command.guidance_mode, 'ilos_line')
        self.assertLess(command.desired_speed, 0.65)
        self.assertGreater(command.desired_speed, 0.35)

    def test_curvature_speed_envelope_can_brake_before_a_far_turn(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=True,
            lattice_enabled=False,
            normal_brake_distance=22.0,
            max_normal_brake_thrust=240.0,
        ))
        controller.set_targets([
            GeoTarget(0.0, 100.0 / 111194.9, math.pi / 2.0),
        ], 'wayfinding')

        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 2.2), 0.05)

        self.assertGreater(command.upcoming_curvature, 0.0)
        self.assertEqual(command.state, 'curve_braking')
        self.assertLess(command.left_thrust + command.right_thrust, 0.0)

    def test_navigation_yaw_rate_reference_cannot_reverse_in_one_frame(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            guidance_enabled=False,
            navigation_yaw_rate_gain=1000.0,
            max_navigation_yaw_acceleration=math.radians(10.0),
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.001, 0.0)], 'wayfinding')

        left_turn = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0, yaw_rate=0.0), 0.05)
        reversed_error = controller.update(
            VesselState(0.0, 0.0, math.pi, 0.0, yaw_rate=0.0), 0.05)

        expected_step = math.radians(0.5)
        self.assertAlmostEqual(
            left_turn.desired_yaw_rate, expected_step, delta=1e-12)
        self.assertGreater(left_turn.right_thrust, left_turn.left_thrust)
        self.assertAlmostEqual(
            reversed_error.desired_yaw_rate, 0.0, delta=1e-12)
        self.assertAlmostEqual(
            reversed_error.left_thrust, reversed_error.right_thrust,
            delta=1e-12,
        )

    def test_navigation_brakes_excessive_yaw_rate(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            navigation_yaw_rate_gain=1000.0,
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.0, 0.001)], 'wayfinding')

        command = controller.update(
            VesselState(
                0.0, 0.0, 0.0, 0.0,
                yaw_rate=math.radians(45.0)),
            0.05,
        )

        self.assertGreater(command.left_thrust, command.right_thrust)
        self.assertLessEqual(
            abs(command.left_thrust - command.right_thrust),
            2.0 * config.max_turn_thrust,
        )
        self.assertEqual(command.desired_speed, 0.0)

    def test_mixer_adapts_to_negative_forward_polarity(self):
        config = ControlConfig(
            forward_thrust_sign=-1.0,
            obstacle_avoidance_enabled=False,
        )
        controller = ControllerCore(config)
        controller.set_targets(
            [GeoTarget(0.001, 0.0, math.pi / 2)], 'wayfinding')
        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)
        self.assertGreater(command.left_thrust, command.right_thrust)

    def test_emergency_avoidance_cannot_cancel_against_target_bearing(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=True,
            obstacle_warning_distance=18.0,
            obstacle_emergency_distance=7.0,
        )
        controller = ControllerCore(config)
        distance = 100.0
        bearing = math.radians(70.0)
        north = math.sin(bearing) * distance / 111194.9
        east = math.cos(bearing) * distance / 111194.9
        controller.set_targets([GeoTarget(north, east)], 'wayfinding')

        command = controller.update(VesselState(
            latitude=0.0,
            longitude=0.0,
            yaw=0.0,
            speed=1.0,
            laser_ranges=(6.0, 6.0),
            laser_angle_min=0.0,
            laser_angle_increment=0.01,
        ), 0.05)

        self.assertEqual(command.state, 'braking')
        self.assertEqual(command.desired_speed, 0.0)
        self.assertGreater(abs(command.heading_error), math.radians(20.0))
        self.assertLess(command.left_thrust + command.right_thrust, 0.0)

    def test_static_boat_pivots_before_accelerating_at_close_obstacle(self):
        controller = ControllerCore(ControlConfig(obstacle_avoidance_enabled=True))
        controller.set_targets([GeoTarget(0.0, 0.001)], 'wayfinding')
        command = controller.update(VesselState(
            latitude=0.0,
            longitude=0.0,
            yaw=0.0,
            speed=0.0,
            laser_ranges=(4.0, 4.0),
            laser_angle_min=0.0,
            laser_angle_increment=0.01,
        ), 0.05)
        self.assertEqual(command.state, 'pivoting')
        self.assertEqual(command.desired_speed, 0.0)
        self.assertAlmostEqual(command.left_thrust + command.right_thrust, 0.0)

    def test_turn_sweep_brakes_reverse_drift_near_a_side_obstacle(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            obstacle_safety_radius=3.0,
            obstacle_path_half_width=2.4,
        ))
        controller.set_targets([GeoTarget(0.001, 0.0)], 'wayfinding')

        command = controller.update(VesselState(
            latitude=0.0,
            longitude=0.0,
            yaw=0.0,
            speed=-0.8,
            obstacle_points=((5.0, -math.pi / 2.0),) * 3,
        ), 0.05)

        self.assertEqual(command.state, 'braking')
        self.assertEqual(command.collision_clearance, 5.0)
        self.assertGreater(command.left_thrust + command.right_thrust, 0.0)
        self.assertEqual(command.desired_yaw_rate, 0.0)

    def test_emergency_braking_survives_a_single_clear_frame(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=True,
            obstacle_clear_hold_time=0.5,
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.0, 0.001)], 'wayfinding')
        blocked = controller.update(VesselState(
            0.0, 0.0, 0.0, 1.0,
            obstacle_points=((5.0, 0.0),),
        ), 0.05)
        one_clear_frame = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)

        self.assertEqual(blocked.state, 'braking')
        self.assertEqual(one_clear_frame.state, 'pivoting')
        self.assertAlmostEqual(
            one_clear_frame.left_thrust + one_clear_frame.right_thrust, 0.0)

    def test_caution_zone_uses_limited_approach_braking(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=True,
            obstacle_emergency_distance=8.5,
            obstacle_brake_time_horizon=2.5,
            obstacle_emergency_time_horizon=0.6,
            obstacle_caution_speed=0.65,
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.0, 0.001)], 'wayfinding')

        command = controller.update(VesselState(
            0.0, 0.0, 0.0, 1.4,
            obstacle_points=((12.0, 0.0),),
        ), 0.05)

        self.assertEqual(command.state, 'approach_braking')
        self.assertLessEqual(command.desired_speed, 0.65)
        self.assertLess(command.left_thrust + command.right_thrust, 0.0)
        self.assertGreaterEqual(
            command.left_thrust + command.right_thrust,
            -2.0 * config.max_normal_brake_thrust,
        )

    def test_backup_is_inhibited_when_rear_clearance_is_blocked(self):
        controller = ControllerCore(
            ControlConfig(obstacle_avoidance_enabled=True))
        controller.set_targets([GeoTarget(0.0, 0.001)], 'wayfinding')
        controller.backup_remaining = 1.0
        ranges = [30.0] * 361
        ranges[1] = 1.0
        ranges[2] = 1.0

        command = controller.update(VesselState(
            0.0, 0.0, 0.0, 0.0,
            laser_ranges=ranges,
            laser_angle_min=-math.pi,
            laser_angle_increment=math.pi / 180.0,
        ), 0.05)

        self.assertEqual(command.state, 'pivoting')
        self.assertAlmostEqual(command.left_thrust + command.right_thrust, 0.0)

    def test_side_obstacle_does_not_cause_a_zero_speed_deadlock(self):
        config = ControlConfig(obstacle_avoidance_enabled=True)
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.0, 0.001)], 'wayfinding')
        command = controller.update(VesselState(
            0.0,
            0.0,
            0.0,
            1.0,
            obstacle_points=((7.0, math.radians(60.0)),) * 3,
        ), 0.05)
        self.assertGreater(command.desired_speed, 0.5)
        self.assertNotEqual(command.state, 'braking')

    def test_mixer_preserves_reverse_surge(self):
        forward = self.controller._mix(500.0, 0.0)
        reverse = self.controller._mix(-500.0, 0.0)
        self.assertEqual(forward, (500.0, 500.0))
        self.assertEqual(reverse, (-500.0, -500.0))

    def test_persistent_no_progress_triggers_backup(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=True,
            obstacle_stuck_timeout=0.5,
            obstacle_backup_duration=0.4,
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.0, 0.001)], 'wayfinding')
        vessel = VesselState(
            0.0,
            0.0,
            0.0,
            0.0,
            obstacle_points=((5.0, 0.0),),
        )
        command = None
        for _ in range(4):
            command = controller.update(vessel, 0.2)
        self.assertEqual(command.state, 'backing_away')
        self.assertLess(command.left_thrust + command.right_thrust, 0.0)

    def test_clear_water_no_progress_does_not_trigger_obstacle_backup(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=True,
            obstacle_stuck_timeout=0.5,
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.0, 0.001)], 'wayfinding')

        command = None
        for _ in range(6):
            command = controller.update(
                VesselState(0.0, 0.0, 0.0, 0.0), 0.2)

        self.assertNotEqual(command.state, 'backing_away')
        self.assertEqual(controller.backup_remaining, 0.0)

    def test_disabling_avoidance_disables_backup_recovery(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            obstacle_stuck_timeout=0.5,
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.0, 0.001)], 'wayfinding')
        command = None
        for _ in range(4):
            command = controller.update(
                VesselState(0.0, 0.0, 0.0, 0.0), 0.2)
        self.assertNotEqual(command.state, 'backing_away')

    def test_new_targets_reset_avoidance_and_recovery_state(self):
        self.controller.avoidance.escape_direction = -1.0
        self.controller.no_progress_elapsed = 3.0
        self.controller.backup_remaining = 1.0
        self.controller.set_targets([GeoTarget(0.0, 0.001)], 'wayfinding')
        self.assertEqual(self.controller.avoidance.escape_direction, 0.0)
        self.assertEqual(self.controller.no_progress_elapsed, 0.0)
        self.assertEqual(self.controller.backup_remaining, 0.0)

    def test_persistent_map_blockage_triggers_lattice_astar_replan(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            lattice_enabled=True,
            lattice_blocked_path_confirmations=2,
            lattice_path_check_stride=1,
        )
        controller = ControllerCore(config)
        target_longitude = math.degrees(30.0 / 6371000.0)
        controller.set_targets(
            [GeoTarget(0.0, target_longitude, 0.0)], 'wayfinding')

        def snapshot(revision, blocked=False):
            width = height = 100
            cells = [False] * (width * height)
            if blocked:
                cells[50 * width + 62] = True
            return OccupancySnapshot(
                origin_east=-50.0,
                origin_north=-50.0,
                resolution=1.0,
                width=width,
                height=height,
                probabilities=tuple(-1 for _ in cells),
                blocked=tuple(cells),
                revision=revision,
            )

        def vessel(grid):
            return VesselState(
                0.0, 0.0, 0.0, 0.5,
                east=0.0, north=0.0, occupancy_grid=grid)

        initial = controller.update(vessel(snapshot(1)), 0.05)
        first_blocked = controller.update(vessel(snapshot(2, True)), 0.05)
        replanned = controller.update(vessel(snapshot(3, True)), 0.05)

        self.assertEqual(initial.guidance_mode, 'lattice_ilos')
        self.assertFalse(first_blocked.guidance_replanned)
        self.assertEqual(first_blocked.lattice_blocked_confirmations, 1)
        self.assertTrue(replanned.guidance_replanned)
        self.assertEqual(replanned.guidance_replan_reason, 'lattice_obstacle')
        self.assertGreater(replanned.lattice_expanded_states, 0)
        self.assertFalse(replanned.lattice_fallback)

    def test_failed_online_lattice_replan_keeps_existing_path(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            lattice_enabled=True,
            lattice_blocked_path_confirmations=2,
            lattice_path_check_stride=1,
            lattice_max_expansions=100,
        )
        controller = ControllerCore(config)
        target_longitude = math.degrees(30.0 / 6371000.0)
        controller.set_targets(
            [GeoTarget(0.0, target_longitude, 0.0)], 'wayfinding')

        def snapshot(revision, blocked):
            width = height = 100
            cells = [blocked] * (width * height)
            if blocked:
                cells[50 * width + 50] = False
            return OccupancySnapshot(
                -50.0, -50.0, 1.0, width, height,
                tuple(-1 for _ in cells), tuple(cells), revision)

        def vessel(grid):
            return VesselState(
                0.0, 0.0, 0.0, 0.5,
                east=0.0, north=0.0, occupancy_grid=grid)

        initial = controller.update(vessel(snapshot(1, False)), 0.05)
        controller.update(vessel(snapshot(2, True)), 0.05)
        rejected = controller.update(vessel(snapshot(3, True)), 0.05)

        self.assertEqual(initial.guidance_mode, 'lattice_ilos')
        self.assertEqual(rejected.guidance_mode, 'lattice_ilos')
        self.assertFalse(rejected.guidance_replanned)
        self.assertFalse(rejected.lattice_fallback)
        self.assertEqual(rejected.path_revision, initial.path_revision)

    def test_failed_horizon_replan_keeps_collision_checked_partial_path(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            lattice_enabled=True,
            lattice_planning_horizon=40.0,
            lattice_replan_distance=6.0,
            guidance_replan_cooldown=5.0,
        )
        controller = ControllerCore(config)
        target_longitude = math.degrees(80.0 / 6371000.0)
        controller.set_targets(
            [GeoTarget(0.0, target_longitude, 0.0)], 'wayfinding')
        width = height = 120
        grid = OccupancySnapshot(
            -60.0, -60.0, 1.0, width, height,
            tuple(-1 for _ in range(width * height)),
            tuple(False for _ in range(width * height)),
            1,
        )
        initial = controller.update(VesselState(
            0.0, 0.0, 0.0, 0.5,
            east=0.0, north=0.0, occupancy_grid=grid), 0.05)
        self.assertTrue(initial.lattice_partial_path)
        revision = initial.path_revision
        controller.ilos.segment_index = len(controller.ilos.points) - 12
        near_horizon_longitude = math.degrees(39.0 / 6371000.0)
        with patch.object(
            controller, '_plan_guidance_path', return_value=False,
        ) as replan:
            rejected = controller.update(VesselState(
                0.0, near_horizon_longitude, 0.0, 0.5,
                east=39.0, north=0.0, occupancy_grid=grid), 0.05)
        self.assertFalse(rejected.guidance_replanned)
        self.assertFalse(rejected.lattice_fallback)
        self.assertEqual(revision, rejected.path_revision)
        self.assertFalse(replan.call_args.kwargs['allow_fallback'])
        self.assertEqual(0.0, rejected.guidance_replan_cooldown_remaining)

    def test_navigation_to_alignment_resets_pid_derivative_state(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            max_alignment_thrust=300.0,
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.0, 0.0, 0.2)], 'wayfinding')
        controller.update(
            VesselState(-0.0001, -0.00005, 0.0, 0.0), 0.05)
        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)
        self.assertEqual(command.state, 'aligning')
        self.assertLess(command.left_thrust, command.right_thrust)

    def test_waypoint_requires_position_heading_and_dwell(self):
        self.controller.set_targets([GeoTarget(0.0, 0.0, math.pi / 2)], 'wayfinding')
        aligning = self.controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)
        self.assertEqual(aligning.state, 'aligning')
        dwell = self.controller.update(
            VesselState(0.0, 0.0, math.pi / 2, 0.0), 0.05)
        self.assertEqual(dwell.state, 'waypoint_dwell')
        done = self.controller.update(
            VesselState(0.0, 0.0, math.pi / 2, 0.0), 0.06)
        self.assertEqual(done.state, 'complete')
        self.assertEqual(done.left_thrust, 0.0)

    def test_final_alignment_uses_reduced_turn_limit(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            max_alignment_thrust=300.0,
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.0, 0.0, math.pi)], 'wayfinding')
        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)
        self.assertEqual(command.state, 'aligning')
        self.assertLessEqual(abs(command.left_thrust), 300.0)
        self.assertLessEqual(abs(command.right_thrust), 300.0)

    def test_final_alignment_halts_when_hull_sweep_clearance_is_unsafe(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=True,
            obstacle_safety_radius=3.0,
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.0, math.pi / 2.0)], 'wayfinding')

        command = controller.update(VesselState(
            0.0,
            0.0,
            0.0,
            0.0,
            yaw_rate=0.0,
            obstacle_points=((1.0, 0.0),),
        ), 0.05)

        self.assertEqual(command.state, 'alignment_blocked')
        self.assertEqual(0.0, command.desired_yaw_rate)
        self.assertEqual(0.0, command.left_thrust)
        self.assertEqual(0.0, command.right_thrust)
        self.assertEqual(1.0, command.collision_clearance)

    def test_final_alignment_caps_requested_yaw_rate(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            max_alignment_thrust=300.0,
            alignment_yaw_rate_gain=1000.0,
            max_alignment_yaw_rate=math.radians(4.0),
            max_alignment_yaw_acceleration=math.radians(1000.0),
        )
        controller = ControllerCore(config)
        controller.set_targets(
            [GeoTarget(0.0, 0.0, math.pi / 2.0)], 'wayfinding')

        command = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0, yaw_rate=0.0), 0.05)

        self.assertEqual(command.state, 'aligning')
        self.assertAlmostEqual(
            command.desired_yaw_rate, math.radians(4.0), delta=1e-6)
        expected_turn = 1000.0 * command.desired_yaw_rate
        self.assertAlmostEqual(command.right_thrust, expected_turn)
        self.assertAlmostEqual(command.left_thrust, -expected_turn)

    def test_final_alignment_rate_reference_cannot_reverse_in_one_frame(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            max_alignment_thrust=300.0,
            alignment_yaw_rate_gain=1000.0,
            max_alignment_yaw_rate=math.radians(4.0),
            max_alignment_yaw_acceleration=math.radians(2.0),
        ))
        controller.set_targets(
            [GeoTarget(0.0, 0.0, math.pi / 2.0)], 'wayfinding')

        left_turn = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0, yaw_rate=0.0), 0.5)
        reversed_error = controller.update(
            VesselState(0.0, 0.0, math.pi, 0.0, yaw_rate=0.0), 0.5)

        self.assertAlmostEqual(
            left_turn.desired_yaw_rate, math.radians(1.0), delta=1e-12)
        self.assertGreater(left_turn.right_thrust, left_turn.left_thrust)
        self.assertAlmostEqual(
            reversed_error.desired_yaw_rate, 0.0, delta=1e-12)
        self.assertAlmostEqual(
            reversed_error.left_thrust,
            reversed_error.right_thrust,
            delta=1e-12,
        )

    def test_alignment_rate_reference_resets_at_state_boundaries(self):
        controller = ControllerCore(ControlConfig(
            obstacle_avoidance_enabled=False,
            max_alignment_yaw_acceleration=math.radians(2.0),
        ))
        target = GeoTarget(0.0, 0.0, math.pi / 2.0)
        controller.set_targets([target], 'wayfinding')
        controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.5)
        self.assertAlmostEqual(
            controller.alignment_yaw_rate_command,
            math.radians(1.0),
            delta=1e-12,
        )

        controller.stop()
        self.assertEqual(controller.alignment_yaw_rate_command, 0.0)
        controller.alignment_yaw_rate_command = math.radians(3.0)
        controller.set_targets([target], 'wayfinding')
        self.assertEqual(controller.alignment_yaw_rate_command, 0.0)

        first_reentry = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.5)
        controller.update(
            VesselState(0.00005, 0.0, 0.0, 0.0), 0.5)
        second_reentry = controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.5)

        self.assertAlmostEqual(
            first_reentry.desired_yaw_rate,
            math.radians(1.0),
            delta=1e-12,
        )
        self.assertAlmostEqual(
            second_reentry.desired_yaw_rate,
            math.radians(1.0),
            delta=1e-12,
        )

    def test_waypoint_dwell_waits_for_low_yaw_rate(self):
        self.controller.set_targets(
            [GeoTarget(0.0, 0.0, 0.0)], 'wayfinding')
        rotating = self.controller.update(
            VesselState(
                0.0,
                0.0,
                0.0,
                0.0,
                yaw_rate=math.radians(10.0),
            ),
            0.05,
        )
        self.assertEqual(rotating.state, 'aligning')
        self.assertGreater(rotating.left_thrust, rotating.right_thrust)
        settled = self.controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0, yaw_rate=0.0), 0.05)
        self.assertEqual(settled.state, 'waypoint_dwell')

    def test_waypoint_dwell_uses_heading_capture_hysteresis(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            waypoint_dwell_time=0.5,
        )
        controller = ControllerCore(config)
        controller.set_targets(
            [GeoTarget(0.0, 0.0, 0.0)], 'wayfinding')

        captured = controller.update(
            VesselState(
                0.0, 0.0, math.radians(-7.0), 0.0,
                yaw_rate=math.radians(2.0)),
            0.1,
        )
        completed = controller.update(
            VesselState(
                0.0, 0.0, math.radians(-10.0), 0.0,
                yaw_rate=math.radians(5.0)),
            0.4,
        )

        self.assertEqual(captured.state, 'waypoint_dwell')
        self.assertEqual(completed.state, 'complete')

    def test_alignment_brakes_rotation_before_crossing_target_yaw(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            max_turn_thrust=300.0,
            max_alignment_thrust=220.0,
            max_alignment_brake_thrust=300.0,
            alignment_yaw_rate_gain=700.0,
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.0, 0.0, 0.0)], 'wayfinding')
        controller.heading_pid.previous_error = -1.0
        controller.heading_pid.filtered_derivative = -10.0
        controller.alignment_active = True

        command = controller.update(
            VesselState(
                0.0,
                0.0,
                math.radians(2.0),
                0.0,
                yaw_rate=math.radians(-30.0),
            ),
            0.05,
        )

        self.assertEqual(command.state, 'aligning')
        self.assertLess(command.left_thrust, command.right_thrust)
        self.assertGreater(abs(command.left_thrust), 220.0)
        self.assertLessEqual(abs(command.left_thrust), 300.0)

    def test_captured_waypoint_uses_exit_hysteresis_during_alignment(self):
        config = ControlConfig(
            obstacle_avoidance_enabled=False,
            position_tolerance=2.5,
            waypoint_exit_tolerance=4.0,
        )
        controller = ControllerCore(config)
        controller.set_targets([GeoTarget(0.0, 0.0, 0.0)], 'wayfinding')

        captured = controller.update(
            VesselState(0.0, 0.0, math.pi / 2.0, 0.0), 0.05)
        within_exit_band = controller.update(
            VesselState(0.000027, 0.0, math.pi / 2.0, 0.0), 0.05)
        outside_exit_band = controller.update(
            VesselState(0.000045, 0.0, -math.pi / 2.0, 0.0), 0.05)

        self.assertEqual(captured.state, 'aligning')
        self.assertEqual(within_exit_band.state, 'aligning')
        self.assertEqual(outside_exit_band.state, 'navigating')

    def test_waypoint_without_yaw_does_not_force_east_alignment(self):
        self.controller.set_targets([GeoTarget(0.0, 0.0, None)], 'wayfinding')
        dwell = self.controller.update(
            VesselState(0.0, 0.0, math.pi, 0.0), 0.05)
        self.assertEqual(dwell.state, 'waypoint_dwell')

    def test_stationkeeping_resumes_after_drift(self):
        self.controller.set_targets([GeoTarget(0.0, 0.0, 0.0)], 'stationkeeping')
        holding = self.controller.update(
            VesselState(0.0, 0.0, 0.0, 0.0), 0.05)
        self.assertEqual(holding.state, 'stationkeeping')
        moving = self.controller.update(
            VesselState(0.0, 0.0001, math.pi, 0.0), 0.05)
        self.assertEqual(moving.state, 'navigating')

    def test_stationkeeping_must_capture_before_using_exit_band(self):
        self.controller.set_targets([GeoTarget(0.0, 0.0, 0.0)], 'stationkeeping')
        about_three_metres_east = 3.0 / 111194.9
        moving = self.controller.update(
            VesselState(0.0, about_three_metres_east, math.pi, 0.0), 0.05)
        self.assertEqual(moving.state, 'navigating')
        self.assertNotEqual(moving.left_thrust, 0.0)


if __name__ == '__main__':
    unittest.main()
