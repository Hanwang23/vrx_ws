"""Collect repeatable controller and official VRX task metrics."""

import argparse
from collections import Counter, defaultdict
import json
import math
import time

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Float32Array, ParamVec
from std_msgs.msg import Float32, String


def _param_value(message: ParamVec, name: str):
    for parameter in message.params:
        if parameter.name != name:
            continue
        value = parameter.value
        if name in ('state', 'name'):
            return value.string_value
        if name == 'num_collisions':
            return value.integer_value
        return value.double_value
    return None


class RegressionMonitor(Node):
    def __init__(
        self,
        timeout: float,
        live_period: float,
        completion_grace: float,
    ) -> None:
        super().__init__('codex_usv_regression_monitor')
        self.timeout = max(1.0, timeout)
        self.live_period = max(0.5, live_period)
        self.completion_grace = max(0.0, completion_grace)
        self.started = time.monotonic()
        self.last_live = 0.0
        self.final = None
        self.completed = False
        self.done = False
        self.state_counts = Counter()
        self.status_samples = 0
        self.xte = defaultdict(lambda: {
            'count': 0, 'sum': 0.0, 'max': 0.0, 'final': 0.0,
        })
        self.max_collisions = 0
        self.collision_samples = 0
        self.max_yaw_rate = 0.0
        self.yaw_rate_samples = 0
        self.max_alignment_yaw_rate = 0.0
        self.max_alignment_command_yaw_rate = 0.0
        self.min_collision_clearance = math.inf
        self.max_path_revision = 0
        self.guidance_replan_count = 0
        self.max_map_known_cells = 0
        self.max_map_occupied_cells = 0
        self.max_lattice_expanded_states = 0
        self.planning_event_count = 0
        self.total_planning_time_ms = 0.0
        self.max_planning_time_ms = 0.0
        self.lattice_path_samples = 0
        self.lattice_fallback_samples = 0
        self.lattice_obstacle_replan_count = 0
        self.official_score = None
        self.official_state = None
        self.official_mean_error = None
        self.official_min_errors = []
        self.xte_count = 0
        self.xte_sum = 0.0
        self.xte_max = 0.0
        self.last_status_time = None
        self.in_place_rotation_s = 0.0
        self.max_continuous_rotation_s = 0.0
        self.current_rotation_s = 0.0
        self.estimator_source_counts = Counter()
        self.max_estimator_fallback_count = 0
        self.max_dynamic_track_count = 0
        self.colregs_active_samples = 0
        self.colregs_encounter_counts = Counter()
        self.max_dynamic_masked_scan_beams = 0
        self.max_dynamic_masked_cloud_tracks = 0
        self.active_target_index = None
        self.active_target_started = None
        self.waypoint_durations = {}
        self.waypoint_alignment_s = defaultdict(float)
        self.complete_observed_at = None
        self.official_metrics_ready_at = None
        self.create_subscription(
            String, '/autonomous_usv/status', self._status_callback, 20)
        self.create_subscription(
            ParamVec, '/vrx/task/info', self._task_callback, 10)
        self.create_subscription(
            Float32, '/vrx/wayfinding/mean_error', self._mean_error_callback, 10)
        self.create_subscription(
            Float32Array, '/vrx/wayfinding/min_errors',
            self._min_errors_callback, 10)
        self.create_timer(0.25, self._timer_callback)

    def _task_callback(self, message: ParamVec) -> None:
        score = _param_value(message, 'score')
        state = _param_value(message, 'state')
        collisions = _param_value(message, 'num_collisions')
        if score is not None and math.isfinite(float(score)):
            self.official_score = float(score)
        if state:
            self.official_state = str(state)
        if collisions is not None:
            self.collision_samples += 1
            self.max_collisions = max(self.max_collisions, int(collisions))

    def _mean_error_callback(self, message: Float32) -> None:
        if math.isfinite(message.data):
            self.official_mean_error = float(message.data)

    def _min_errors_callback(self, message: Float32Array) -> None:
        self.official_min_errors = [
            float(value) for value in message.data if math.isfinite(value)]
        if self.completed and self.final:
            target_count = int(self.final.get('target_count') or 0)
            if (
                target_count > 0
                and len(self.official_min_errors) == target_count
                and self.official_metrics_ready_at is None
            ):
                self.official_metrics_ready_at = time.monotonic()

    def _status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self.status_samples += 1
        self.final = status
        now = time.monotonic()
        state = str(status.get('state', 'unknown'))
        target_index = int(status.get('target_index') or 0)
        target_count = int(status.get('target_count') or 0)
        if target_count > 0 and target_index < target_count:
            if self.active_target_index is None:
                self.active_target_index = target_index
                self.active_target_started = now
            elif target_index != self.active_target_index:
                if self.active_target_started is not None:
                    self.waypoint_durations[str(self.active_target_index)] = (
                        now - self.active_target_started)
                self.active_target_index = target_index
                self.active_target_started = now
        self.state_counts[state] += 1
        estimator_source = str(status.get('estimator_source', 'unknown'))
        self.estimator_source_counts[estimator_source] += 1
        self.max_estimator_fallback_count = max(
            self.max_estimator_fallback_count,
            int(status.get('estimator_fallback_count') or 0))
        self.max_dynamic_track_count = max(
            self.max_dynamic_track_count,
            int(status.get('dynamic_track_count') or 0))
        if status.get('colregs_active'):
            self.colregs_active_samples += 1
            self.colregs_encounter_counts[str(
                status.get('colregs_encounter') or 'unknown')] += 1
        self.max_dynamic_masked_scan_beams = max(
            self.max_dynamic_masked_scan_beams,
            int(status.get('dynamic_masked_scan_beams') or 0))
        self.max_dynamic_masked_cloud_tracks = max(
            self.max_dynamic_masked_cloud_tracks,
            int(status.get('dynamic_masked_cloud_tracks') or 0))

        dt = 0.0
        if self.last_status_time is not None:
            dt = min(0.25, max(0.0, now - self.last_status_time))
        self.last_status_time = now
        rotating_in_place = (
            state in ('aligning', 'pivoting')
            and abs(float(status.get('speed_mps') or 0.0)) <= 0.25
            and abs(float(status.get('desired_speed_mps') or 0.0)) <= 0.05
        )
        if rotating_in_place:
            self.in_place_rotation_s += dt
            self.current_rotation_s += dt
            if target_index < target_count:
                self.waypoint_alignment_s[str(target_index)] += dt
            self.max_continuous_rotation_s = max(
                self.max_continuous_rotation_s, self.current_rotation_s)
        else:
            self.current_rotation_s = 0.0
        if status.get('num_collisions') is not None:
            self.collision_samples += 1
            self.max_collisions = max(
                self.max_collisions, int(status['num_collisions']))
        yaw_rate_value = status.get('yaw_rate_deg_s')
        if yaw_rate_value is not None and math.isfinite(float(yaw_rate_value)):
            self.yaw_rate_samples += 1
            yaw_rate = abs(float(yaw_rate_value))
            self.max_yaw_rate = max(self.max_yaw_rate, yaw_rate)
            if state == 'aligning':
                self.max_alignment_yaw_rate = max(
                    self.max_alignment_yaw_rate, yaw_rate)
                desired_yaw_rate = status.get('desired_yaw_rate_deg_s')
                if (
                    desired_yaw_rate is not None
                    and math.isfinite(float(desired_yaw_rate))
                ):
                    self.max_alignment_command_yaw_rate = max(
                        self.max_alignment_command_yaw_rate,
                        abs(float(desired_yaw_rate)),
                    )
        clearance = status.get('collision_clearance_m')
        if clearance is not None:
            self.min_collision_clearance = min(
                self.min_collision_clearance, float(clearance))
        self.max_path_revision = max(
            self.max_path_revision, int(status.get('path_revision') or 0))
        if status.get('guidance_replanned'):
            self.guidance_replan_count += 1
            if status.get('guidance_replan_reason') == 'lattice_obstacle':
                self.lattice_obstacle_replan_count += 1
        self.max_map_known_cells = max(
            self.max_map_known_cells, int(status.get('map_known_cells') or 0))
        self.max_map_occupied_cells = max(
            self.max_map_occupied_cells,
            int(status.get('map_occupied_cells') or 0))
        self.max_lattice_expanded_states = max(
            self.max_lattice_expanded_states,
            int(status.get('lattice_expanded_states') or 0))
        planning_time_ms = max(
            0.0, float(status.get('lattice_planning_time_ms') or 0.0))
        if planning_time_ms > 0.0:
            self.planning_event_count += 1
            self.total_planning_time_ms += planning_time_ms
            self.max_planning_time_ms = max(
                self.max_planning_time_ms, planning_time_ms)
        if status.get('guidance_mode') == 'lattice_ilos':
            self.lattice_path_samples += 1
        if status.get('lattice_fallback'):
            self.lattice_fallback_samples += 1

        xte = status.get('cross_track_error_m')
        if status.get('path_valid') and xte is not None:
            value = abs(float(xte))
            record = self.xte[target_index]
            record['count'] += 1
            record['sum'] += value
            record['max'] = max(record['max'], value)
            record['final'] = value
            self.xte_count += 1
            self.xte_sum += value
            self.xte_max = max(self.xte_max, value)

        self.completed = (
            target_count > 0
            and target_index >= target_count
            and state == 'complete'
        )
        if self.completed:
            if self.active_target_index is not None:
                if self.active_target_started is not None:
                    self.waypoint_durations[str(self.active_target_index)] = (
                        now - self.active_target_started)
                self.active_target_index = None
                self.active_target_started = None
            if self.complete_observed_at is None:
                self.complete_observed_at = now
            if (
                len(self.official_min_errors) == target_count
                and self.official_metrics_ready_at is None
            ):
                self.official_metrics_ready_at = now
        if now - self.last_live >= self.live_period:
            self.last_live = now
            print(
                'LIVE '
                f't={now - self.started:.1f}s '
                f'wp={min(target_index + 1, target_count)}/{target_count} '
                f'state={state} dist={status.get("distance_m")} '
                f'remain={status.get("path_remaining_m")} '
                f'xte={float(xte or 0.0):+.2f} '
                f'yaw_rate={float(status.get("yaw_rate_deg_s") or 0.0):+.1f} '
                f'hits={self.max_collisions}',
                flush=True,
            )
    def _timer_callback(self) -> None:
        now = time.monotonic()
        if self.completed and self.complete_observed_at is not None:
            if (
                self.official_metrics_ready_at is not None
                and now - self.official_metrics_ready_at
                >= self.completion_grace
            ):
                self.done = True
            elif now - self.complete_observed_at >= max(
                10.0, self.completion_grace + 5.0
            ):
                self.done = True
        if now - self.started >= self.timeout:
            self.done = True

    def summary(self):
        xte_summary = {}
        for index, record in sorted(self.xte.items()):
            count = max(1, record['count'])
            xte_summary[str(index)] = {
                'max_abs_m': record['max'],
                'mean_abs_m': record['sum'] / count,
                'final_abs_m': record['final'],
            }
        return {
            'completed': self.completed,
            'elapsed_wall_s': time.monotonic() - self.started,
            'status_samples': self.status_samples,
            'num_collisions': (
                self.max_collisions if self.collision_samples else None),
            'state_counts': dict(self.state_counts),
            'xte': xte_summary,
            'max_yaw_rate_deg_s': (
                self.max_yaw_rate if self.yaw_rate_samples else None),
            'max_alignment_yaw_rate_deg_s': (
                self.max_alignment_yaw_rate
                if self.yaw_rate_samples else None),
            'max_alignment_command_yaw_rate_deg_s': (
                self.max_alignment_command_yaw_rate
                if self.yaw_rate_samples else None),
            'min_collision_clearance_m': (
                self.min_collision_clearance
                if math.isfinite(self.min_collision_clearance) else None),
            'max_path_revision': self.max_path_revision,
            'guidance_replan_count': self.guidance_replan_count,
            'max_map_known_cells': self.max_map_known_cells,
            'max_map_occupied_cells': self.max_map_occupied_cells,
            'max_lattice_expanded_states': self.max_lattice_expanded_states,
            'planning_event_count': self.planning_event_count,
            'total_planning_time_ms': self.total_planning_time_ms,
            'mean_planning_time_ms': (
                self.total_planning_time_ms / self.planning_event_count
                if self.planning_event_count else 0.0),
            'max_planning_time_ms': self.max_planning_time_ms,
            'mean_abs_cross_track_error_m': (
                self.xte_sum / self.xte_count if self.xte_count else None),
            'max_abs_cross_track_error_m': (
                self.xte_max if self.xte_count else None),
            'in_place_rotation_s': self.in_place_rotation_s,
            'max_continuous_rotation_s': self.max_continuous_rotation_s,
            'waypoint_durations_s': dict(self.waypoint_durations),
            'waypoint_alignment_s': dict(self.waypoint_alignment_s),
            'max_waypoint_duration_s': (
                max(self.waypoint_durations.values())
                if self.waypoint_durations else None),
            'estimator_source_counts': dict(self.estimator_source_counts),
            'max_estimator_fallback_count': (
                self.max_estimator_fallback_count),
            'max_dynamic_track_count': self.max_dynamic_track_count,
            'colregs_active_samples': self.colregs_active_samples,
            'colregs_encounter_counts': dict(self.colregs_encounter_counts),
            'max_dynamic_masked_scan_beams': (
                self.max_dynamic_masked_scan_beams),
            'max_dynamic_masked_cloud_tracks': (
                self.max_dynamic_masked_cloud_tracks),
            'lattice_path_samples': self.lattice_path_samples,
            'lattice_fallback_samples': self.lattice_fallback_samples,
            'lattice_obstacle_replan_count': (
                self.lattice_obstacle_replan_count),
            'official_state': self.official_state,
            'official_score': self.official_score,
            'official_mean_error_m': self.official_mean_error,
            'official_min_errors_m': self.official_min_errors,
            'final': self.final,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument('--live-period', type=float, default=5.0)
    parser.add_argument('--completion-grace', type=float, default=2.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    monitor = RegressionMonitor(
        args.timeout, args.live_period, args.completion_grace)
    try:
        try:
            while rclpy.ok() and not monitor.done:
                rclpy.spin_once(monitor, timeout_sec=0.25)
        except KeyboardInterrupt:
            monitor.done = True
        print(
            'SUMMARY ' + json.dumps(monitor.summary(), separators=(',', ':')),
            flush=True,
        )
        return 0 if monitor.completed and monitor.max_collisions == 0 else 2
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
