import unittest

from codex_usv_controller.evaluation import (
    EvaluationThresholds,
    aggregate_trials,
    analyze_trial,
    parse_summary_lines,
)
from codex_usv_controller.evaluation_runner import (
    STALE_PROCESS_PATTERNS,
    _matching_processes,
)


class EvaluationTests(unittest.TestCase):
    def test_stale_cleanup_covers_every_long_running_codex_process(self):
        patterns = '\n'.join(STALE_PROCESS_PATTERNS)
        for process_name in (
            'run_evaluation',
            'regression_monitor',
            'autonomous_usv',
            'moving_target',
            'gnss_odometry_adapter',
            'robot_localization/ekf_node',
            'gz sim',
            'parameter_bridge',
        ):
            self.assertIn(process_name, patterns)

    def valid_summary(self):
        return {
            'completed': True,
            'elapsed_wall_s': 115.0,
            'status_samples': 100,
            'num_collisions': 0,
            'official_score': 1.1,
            'official_mean_error_m': 0.95,
            'official_min_errors_m': [0.3, 1.1, 1.4],
            'max_yaw_rate_deg_s': 12.0,
            'max_alignment_yaw_rate_deg_s': 4.2,
            'max_alignment_command_yaw_rate_deg_s': 4.0,
            'min_collision_clearance_m': 4.2,
            'mean_abs_cross_track_error_m': 0.8,
            'max_abs_cross_track_error_m': 2.1,
            'max_planning_time_ms': 8.0,
            'mean_planning_time_ms': 3.0,
            'in_place_rotation_s': 4.0,
            'max_continuous_rotation_s': 1.5,
            'waypoint_durations_s': {'0': 35.0, '1': 40.0, '2': 38.0},
            'max_waypoint_duration_s': 40.0,
            'max_estimator_fallback_count': 0,
            'max_map_known_cells': 20000,
            'max_map_occupied_cells': 5,
            'lattice_path_samples': 100,
            'lattice_fallback_samples': 0,
            'max_lattice_expanded_states': 12,
            'max_dynamic_track_count': 2,
            'estimator_source_counts': {'robot_localization': 100},
            'final': {
                'ekf_enabled': True,
                'ekf_healthy': True,
                'robot_localization_enabled': True,
                'target_count': 3,
            },
        }

    def test_valid_trial_passes(self):
        verdict = analyze_trial(self.valid_summary(), EvaluationThresholds())
        self.assertTrue(verdict['passed'])
        self.assertEqual(verdict['issues'], [])
        self.assertEqual(
            4.2, verdict['metrics']['min_collision_clearance_m'])

    def test_every_failed_gate_is_explained(self):
        summary = self.valid_summary()
        summary.update({
            'completed': False,
            'elapsed_wall_s': 700.0,
            'status_samples': 2,
            'num_collisions': 1,
            'official_mean_error_m': 4.0,
            'official_min_errors_m': [1.0, 5.0],
            'max_yaw_rate_deg_s': 25.0,
        })
        verdict = analyze_trial(summary, EvaluationThresholds())
        self.assertFalse(verdict['passed'])
        self.assertEqual(len(verdict['issues']), 7)

    def test_missing_official_metrics_fail_closed(self):
        summary = self.valid_summary()
        summary.pop('official_mean_error_m')
        summary.pop('official_min_errors_m')
        verdict = analyze_trial(summary, EvaluationThresholds())
        self.assertFalse(verdict['passed'])
        self.assertIn('official mean error was not received', verdict['issues'])

    def test_missing_safety_metrics_fail_closed(self):
        summary = self.valid_summary()
        summary.pop('num_collisions')
        summary.pop('max_yaw_rate_deg_s')
        summary.pop('final')
        verdict = analyze_trial(summary, EvaluationThresholds())
        self.assertFalse(verdict['passed'])
        self.assertIn('collision count is missing', verdict['issues'])
        self.assertIn('peak yaw rate is missing', verdict['issues'])
        self.assertIn('final controller status is missing', verdict['issues'])

    def test_aggregate_marks_missing_collision_data(self):
        summary = self.valid_summary()
        summary.pop('num_collisions')
        report = {
            'summary': summary,
            'verdict': analyze_trial(summary, EvaluationThresholds()),
        }
        aggregate = aggregate_trials([report], EvaluationThresholds())
        self.assertFalse(aggregate['collision_data_complete'])
        self.assertIsNone(aggregate['total_collisions'])

    def test_process_matching_ignores_runner_ancestry(self):
        matches = _matching_processes([
            '10 ros2 run codex_usv_controller run_evaluation',
            '11 gz sim -r wayfinding_task.sdf',
            'not-a-pid parameter_bridge',
        ], ignored_pids=(10,))
        self.assertEqual([(11, 'gz sim -r wayfinding_task.sdf')], matches)

    def test_summary_parser_uses_last_valid_record(self):
        summary = parse_summary_lines([
            'noise', 'SUMMARY {bad', 'SUMMARY {"completed":false}',
            'SUMMARY {"completed":true}',
        ])
        self.assertTrue(summary['completed'])

    def test_aggregate_requires_all_trials_to_pass(self):
        thresholds = EvaluationThresholds()
        good = self.valid_summary()
        bad = dict(good, completed=False)
        reports = [
            {'summary': good, 'verdict': analyze_trial(good, thresholds)},
            {'summary': bad, 'verdict': analyze_trial(bad, thresholds)},
        ]
        aggregate = aggregate_trials(reports, thresholds)
        self.assertFalse(aggregate['passed'])
        self.assertEqual(aggregate['passed_count'], 1)

    def test_stress_profile_requires_real_astar_expansion(self):
        summary = self.valid_summary()
        summary['max_lattice_expanded_states'] = 0
        thresholds = EvaluationThresholds(min_lattice_expanded_states=1)
        verdict = analyze_trial(summary, thresholds)
        self.assertFalse(verdict['passed'])
        self.assertIn('expanded at most 0 states', verdict['issues'][-1])

    def test_persistent_lattice_fallback_fails_closed(self):
        summary = self.valid_summary()
        summary['lattice_fallback_samples'] = 1
        verdict = analyze_trial(summary, EvaluationThresholds())
        self.assertFalse(verdict['passed'])
        self.assertIn('fallback persisted for 1 samples', verdict['issues'][-1])

    def test_eight_waypoint_profile_rejects_wrong_controller_count(self):
        summary = self.valid_summary()
        summary['official_min_errors_m'] = [0.5] * 8
        thresholds = EvaluationThresholds(expected_waypoint_count=8)
        verdict = analyze_trial(summary, thresholds)
        self.assertFalse(verdict['passed'])
        self.assertTrue(any(
            'controller loaded 3 waypoints' in issue
            for issue in verdict['issues']))

    def test_eight_waypoint_profile_requires_all_official_errors(self):
        summary = self.valid_summary()
        summary['final']['target_count'] = 8
        summary['official_min_errors_m'] = [0.5] * 7
        thresholds = EvaluationThresholds(expected_waypoint_count=8)
        verdict = analyze_trial(summary, thresholds)
        self.assertFalse(verdict['passed'])
        self.assertTrue(any(
            'contain 7 waypoint errors' in issue
            for issue in verdict['issues']))

    def test_multi_waypoint_operational_limits_fail_closed(self):
        summary = self.valid_summary()
        summary['final']['target_count'] = 8
        summary['official_min_errors_m'] = [0.5] * 8
        summary['max_alignment_yaw_rate_deg_s'] = 7.0
        summary['max_alignment_command_yaw_rate_deg_s'] = 7.0
        summary['max_continuous_rotation_s'] = 61.0
        summary['max_waypoint_duration_s'] = 241.0
        summary['max_estimator_fallback_count'] = 2
        thresholds = EvaluationThresholds(
            expected_waypoint_count=8,
            max_alignment_yaw_rate_deg_s=6.0,
            max_alignment_command_yaw_rate_deg_s=6.0,
            max_continuous_rotation_s=60.0,
            max_waypoint_duration_s=240.0,
            max_estimator_fallback_count=0,
        )
        verdict = analyze_trial(summary, thresholds)
        self.assertFalse(verdict['passed'])
        self.assertEqual(5, len(verdict['issues']))

    def test_colregs_profile_requires_a_real_rule_activation(self):
        summary = self.valid_summary()
        summary['colregs_active_samples'] = 0
        verdict = analyze_trial(
            summary, EvaluationThresholds(min_colregs_active_samples=20))
        self.assertFalse(verdict['passed'])
        self.assertIn('COLREGs was active for only 0 samples', verdict['issues'][-1])

    def test_colregs_profile_requires_both_dynamic_targets(self):
        summary = self.valid_summary()
        summary['max_dynamic_track_count'] = 1
        verdict = analyze_trial(
            summary, EvaluationThresholds(min_dynamic_track_count=2))
        self.assertFalse(verdict['passed'])
        self.assertIn(
            'dynamic tracker observed at most 1 targets', verdict['issues'][-1])

    def test_aggregate_exposes_requested_operational_metrics(self):
        thresholds = EvaluationThresholds()
        summary = self.valid_summary()
        report = {
            'summary': summary,
            'verdict': analyze_trial(summary, thresholds),
        }
        aggregate = aggregate_trials([report], thresholds)
        self.assertEqual(1.0, aggregate['completion_rate'])
        self.assertEqual(4.2, aggregate['minimum_clearance_m'])
        self.assertEqual(8.0, aggregate['worst_planning_time_ms'])
        self.assertEqual(0.8, aggregate['mean_abs_cross_track_error_m'])
        self.assertEqual(4.0, aggregate['max_in_place_rotation_s'])


if __name__ == '__main__':
    unittest.main()
