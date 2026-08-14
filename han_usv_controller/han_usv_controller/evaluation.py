"""Deterministic pass/fail analysis for VRX regression runs."""

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence


@dataclass(frozen=True)
class EvaluationThresholds:
    """Acceptance limits for one official Wayfinding run."""

    max_wall_time_s: float = 600.0
    max_collisions: int = 0
    max_mean_error_m: float = 2.5
    max_waypoint_error_m: float = 3.0
    max_yaw_rate_deg_s: float = 20.0
    min_status_samples: int = 20
    min_map_known_cells: int = 500
    min_lattice_samples: int = 20
    min_lattice_expanded_states: int = 0
    max_lattice_fallback_samples: int = 0
    min_robot_localization_samples: int = 20
    min_colregs_active_samples: int = 0
    min_dynamic_track_count: int = 0
    expected_waypoint_count: int = 0
    max_alignment_yaw_rate_deg_s: float = 0.0
    max_alignment_command_yaw_rate_deg_s: float = 0.0
    max_continuous_rotation_s: float = 0.0
    max_waypoint_duration_s: float = 0.0
    max_estimator_fallback_count: int = -1


def _finite_float(value: Any) -> Optional[float]:
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return None
    return converted if math.isfinite(converted) else None


def _integer(value: Any) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def analyze_trial(
    summary: Mapping[str, Any],
    thresholds: EvaluationThresholds,
) -> Dict[str, Any]:
    """Return a stable verdict and actionable issues for a monitor summary."""
    issues: List[str] = []
    completed = bool(summary.get('completed'))
    if not completed:
        issues.append('official task did not reach the controller complete state')

    collisions = _integer(summary.get('num_collisions'))
    if collisions is None:
        issues.append('collision count is missing')
    elif collisions > thresholds.max_collisions:
        issues.append(
            f'collision count {collisions} exceeds {thresholds.max_collisions}')

    elapsed = _finite_float(summary.get('elapsed_wall_s'))
    if elapsed is None:
        issues.append('wall-clock duration is missing')
    elif elapsed > thresholds.max_wall_time_s:
        issues.append(
            f'wall-clock duration {elapsed:.1f}s exceeds '
            f'{thresholds.max_wall_time_s:.1f}s')

    status_samples = int(summary.get('status_samples') or 0)
    if status_samples < thresholds.min_status_samples:
        issues.append(
            f'only {status_samples} controller status samples were received')

    final_value = summary.get('final')
    if not isinstance(final_value, Mapping) or not final_value:
        issues.append('final controller status is missing')
        final: Mapping[str, Any] = {}
    else:
        final = final_value
    expected_count = thresholds.expected_waypoint_count
    final_target_count = _integer(final.get('target_count'))
    if expected_count > 0 and final_target_count != expected_count:
        issues.append(
            f'controller loaded {final_target_count} waypoints; expected '
            f'{expected_count}')
    if final.get('ekf_enabled') is not True:
        issues.append('custom EKF was not enabled at the end of the run')
    elif final.get('ekf_healthy') is not True:
        issues.append('EKF was not healthy at the end of the run')
    max_map_known = int(summary.get('max_map_known_cells') or 0)
    if max_map_known < thresholds.min_map_known_cells:
        issues.append(
            f'rolling grid observed only {max_map_known} cells; expected at least '
            f'{thresholds.min_map_known_cells}')
    lattice_samples = int(summary.get('lattice_path_samples') or 0)
    if lattice_samples < thresholds.min_lattice_samples:
        issues.append(
            f'lattice guidance produced only {lattice_samples} status samples')
    expanded_states = int(summary.get('max_lattice_expanded_states') or 0)
    if expanded_states < thresholds.min_lattice_expanded_states:
        issues.append(
            f'lattice A* expanded at most {expanded_states} states; expected at '
            f'least {thresholds.min_lattice_expanded_states}')
    fallback_samples = int(summary.get('lattice_fallback_samples') or 0)
    if fallback_samples > thresholds.max_lattice_fallback_samples:
        issues.append(
            f'lattice fallback persisted for {fallback_samples} samples; maximum '
            f'is {thresholds.max_lattice_fallback_samples}')

    estimator_counts = summary.get('estimator_source_counts') or {}
    robot_localization_samples = int(
        estimator_counts.get('robot_localization') or 0)
    if (
        thresholds.min_robot_localization_samples > 0
        and final.get('robot_localization_enabled') is not True
    ):
        issues.append('robot_localization was not enabled at the end of the run')
    elif robot_localization_samples < thresholds.min_robot_localization_samples:
        issues.append(
            'robot_localization supplied only '
            f'{robot_localization_samples} status samples')
    colregs_active_samples = int(
        summary.get('colregs_active_samples') or 0)
    if colregs_active_samples < thresholds.min_colregs_active_samples:
        issues.append(
            f'COLREGs was active for only {colregs_active_samples} samples; '
            f'expected at least {thresholds.min_colregs_active_samples}')
    max_dynamic_track_count = int(
        summary.get('max_dynamic_track_count') or 0)
    if max_dynamic_track_count < thresholds.min_dynamic_track_count:
        issues.append(
            f'dynamic tracker observed at most {max_dynamic_track_count} targets; '
            f'expected at least {thresholds.min_dynamic_track_count}')

    mean_error = _finite_float(summary.get('official_mean_error_m'))
    if mean_error is None:
        issues.append('official mean error was not received')
    elif mean_error > thresholds.max_mean_error_m:
        issues.append(
            f'official mean error {mean_error:.3f}m exceeds '
            f'{thresholds.max_mean_error_m:.3f}m')

    waypoint_errors = summary.get('official_min_errors_m') or []
    finite_errors = [
        error for error in (_finite_float(value) for value in waypoint_errors)
        if error is not None
    ]
    if not finite_errors:
        issues.append('official per-waypoint errors were not received')
    elif expected_count > 0 and len(finite_errors) != expected_count:
        issues.append(
            f'official metrics contain {len(finite_errors)} waypoint errors; '
            f'expected {expected_count}')
    elif max(finite_errors) > thresholds.max_waypoint_error_m:
        issues.append(
            f'worst waypoint error {max(finite_errors):.3f}m exceeds '
            f'{thresholds.max_waypoint_error_m:.3f}m')

    max_yaw_rate = _finite_float(summary.get('max_yaw_rate_deg_s'))
    if max_yaw_rate is None:
        issues.append('peak yaw rate is missing')
    elif max_yaw_rate > thresholds.max_yaw_rate_deg_s:
        issues.append(
            f'peak yaw rate {max_yaw_rate:.2f}deg/s exceeds '
            f'{thresholds.max_yaw_rate_deg_s:.2f}deg/s')

    max_alignment_yaw_rate = _finite_float(
        summary.get('max_alignment_yaw_rate_deg_s'))
    if thresholds.max_alignment_yaw_rate_deg_s > 0.0:
        if max_alignment_yaw_rate is None:
            issues.append('peak alignment yaw rate is missing')
        elif (
            max_alignment_yaw_rate
            > thresholds.max_alignment_yaw_rate_deg_s
        ):
            issues.append(
                'peak alignment yaw rate '
                f'{max_alignment_yaw_rate:.2f}deg/s exceeds '
                f'{thresholds.max_alignment_yaw_rate_deg_s:.2f}deg/s')

    max_alignment_command_yaw_rate = _finite_float(
        summary.get('max_alignment_command_yaw_rate_deg_s'))
    if thresholds.max_alignment_command_yaw_rate_deg_s > 0.0:
        if max_alignment_command_yaw_rate is None:
            issues.append('peak commanded alignment yaw rate is missing')
        elif (
            max_alignment_command_yaw_rate
            > thresholds.max_alignment_command_yaw_rate_deg_s
        ):
            issues.append(
                'peak commanded alignment yaw rate '
                f'{max_alignment_command_yaw_rate:.2f}deg/s exceeds '
                f'{thresholds.max_alignment_command_yaw_rate_deg_s:.2f}deg/s')

    continuous_rotation = _finite_float(
        summary.get('max_continuous_rotation_s'))
    if thresholds.max_continuous_rotation_s > 0.0:
        if continuous_rotation is None:
            issues.append('longest continuous in-place rotation is missing')
        elif continuous_rotation > thresholds.max_continuous_rotation_s:
            issues.append(
                f'continuous in-place rotation {continuous_rotation:.1f}s '
                f'exceeds {thresholds.max_continuous_rotation_s:.1f}s')

    max_waypoint_duration = _finite_float(
        summary.get('max_waypoint_duration_s'))
    if thresholds.max_waypoint_duration_s > 0.0:
        if max_waypoint_duration is None:
            issues.append('per-waypoint duration metrics are missing')
        elif max_waypoint_duration > thresholds.max_waypoint_duration_s:
            issues.append(
                f'slowest waypoint took {max_waypoint_duration:.1f}s; maximum '
                f'is {thresholds.max_waypoint_duration_s:.1f}s')

    estimator_fallback_count = int(
        summary.get('max_estimator_fallback_count') or 0)
    if (
        thresholds.max_estimator_fallback_count >= 0
        and estimator_fallback_count
        > thresholds.max_estimator_fallback_count
    ):
        issues.append(
            f'estimator fallback count {estimator_fallback_count} exceeds '
            f'{thresholds.max_estimator_fallback_count}')

    return {
        'passed': not issues,
        'issues': issues,
        'metrics': {
            'completed': completed,
            'elapsed_wall_s': elapsed,
            'num_collisions': collisions,
            'official_score': _finite_float(summary.get('official_score')),
            'official_mean_error_m': mean_error,
            'official_min_errors_m': finite_errors,
            'max_yaw_rate_deg_s': max_yaw_rate,
            'max_alignment_yaw_rate_deg_s': max_alignment_yaw_rate,
            'max_alignment_command_yaw_rate_deg_s': (
                max_alignment_command_yaw_rate),
            'status_samples': status_samples,
            'max_map_known_cells': max_map_known,
            'max_map_occupied_cells': int(
                summary.get('max_map_occupied_cells') or 0),
            'lattice_path_samples': lattice_samples,
            'max_lattice_expanded_states': expanded_states,
            'lattice_fallback_samples': fallback_samples,
            'robot_localization_samples': robot_localization_samples,
            'colregs_active_samples': colregs_active_samples,
            'max_dynamic_track_count': max_dynamic_track_count,
            'min_collision_clearance_m': _finite_float(
                summary.get('min_collision_clearance_m')),
            'mean_abs_cross_track_error_m': _finite_float(
                summary.get('mean_abs_cross_track_error_m')),
            'max_abs_cross_track_error_m': _finite_float(
                summary.get('max_abs_cross_track_error_m')),
            'max_planning_time_ms': _finite_float(
                summary.get('max_planning_time_ms')),
            'mean_planning_time_ms': _finite_float(
                summary.get('mean_planning_time_ms')),
            'in_place_rotation_s': _finite_float(
                summary.get('in_place_rotation_s')),
            'max_continuous_rotation_s': _finite_float(
                summary.get('max_continuous_rotation_s')),
            'waypoint_durations_s': dict(
                summary.get('waypoint_durations_s') or {}),
            'max_waypoint_duration_s': max_waypoint_duration,
            'max_estimator_fallback_count': estimator_fallback_count,
            'loaded_waypoint_count': final_target_count,
        },
    }


def aggregate_trials(
    trial_reports: Sequence[Mapping[str, Any]],
    thresholds: EvaluationThresholds,
) -> Dict[str, Any]:
    """Summarize repeated trials without hiding an individual failure."""
    passed = [bool(report.get('verdict', {}).get('passed')) for report in trial_reports]
    mean_errors = [
        _finite_float(report.get('summary', {}).get('official_mean_error_m'))
        for report in trial_reports
    ]
    finite_mean_errors = [value for value in mean_errors if value is not None]
    durations = [
        _finite_float(report.get('summary', {}).get('elapsed_wall_s'))
        for report in trial_reports
    ]
    finite_durations = [value for value in durations if value is not None]
    def summary_values(name: str) -> List[float]:
        values = [
            _finite_float(report.get('summary', {}).get(name))
            for report in trial_reports
        ]
        return [value for value in values if value is not None]

    collision_counts = [
        _integer(report.get('summary', {}).get('num_collisions'))
        for report in trial_reports
    ]
    collision_data_complete = all(value is not None for value in collision_counts)
    finite_collision_counts = [
        value for value in collision_counts if value is not None]
    clearances = summary_values('min_collision_clearance_m')
    planning_maxima = summary_values('max_planning_time_ms')
    planning_means = summary_values('mean_planning_time_ms')
    cross_track_means = summary_values('mean_abs_cross_track_error_m')
    cross_track_maxima = summary_values('max_abs_cross_track_error_m')
    yaw_rates = summary_values('max_yaw_rate_deg_s')
    alignment_yaw_rates = summary_values('max_alignment_yaw_rate_deg_s')
    alignment_command_yaw_rates = summary_values(
        'max_alignment_command_yaw_rate_deg_s')
    rotation_times = summary_values('in_place_rotation_s')
    continuous_rotation_times = summary_values('max_continuous_rotation_s')
    waypoint_duration_maxima = summary_values('max_waypoint_duration_s')
    heading_error_means = summary_values('mean_abs_heading_error_deg')
    heading_error_rms = summary_values('rms_heading_error_deg')
    yaw_rate_error_means = summary_values(
        'mean_abs_yaw_rate_tracking_error_deg_s')
    yaw_rate_error_rms = summary_values(
        'rms_yaw_rate_tracking_error_deg_s')
    turn_efforts = summary_values('integrated_abs_turn_thrust')
    thruster_efforts = summary_values('integrated_abs_thruster_command')
    turn_variation_rates = summary_values('turn_thrust_variation_rate')
    saturation_fractions = summary_values('turn_saturation_fraction')
    waypoint_error_values = [
        value
        for report in trial_reports
        for value in (
            _finite_float(item)
            for item in report.get('summary', {}).get(
                'official_min_errors_m', [])
        )
        if value is not None
    ]
    count = len(trial_reports)
    return {
        'passed': bool(trial_reports) and all(passed),
        'trial_count': count,
        'passed_count': sum(passed),
        'completion_rate': (
            sum(bool(report.get('summary', {}).get('completed'))
                for report in trial_reports) / count
            if count else 0.0),
        'collision_data_complete': collision_data_complete,
        'collision_trial_count': (
            sum(value > 0 for value in finite_collision_counts)
            if collision_data_complete else None),
        'total_collisions': (
            sum(finite_collision_counts) if collision_data_complete else None),
        'thresholds': asdict(thresholds),
        'mean_official_error_m': (
            mean(finite_mean_errors) if finite_mean_errors else None),
        'max_official_error_m': (
            max(finite_mean_errors) if finite_mean_errors else None),
        'worst_waypoint_error_m': (
            max(waypoint_error_values) if waypoint_error_values else None),
        'mean_wall_time_s': mean(finite_durations) if finite_durations else None,
        'minimum_clearance_m': min(clearances) if clearances else None,
        'mean_peak_planning_time_ms': (
            mean(planning_maxima) if planning_maxima else None),
        'worst_planning_time_ms': (
            max(planning_maxima) if planning_maxima else None),
        'mean_planning_time_ms': (
            mean(planning_means) if planning_means else None),
        'mean_abs_cross_track_error_m': (
            mean(cross_track_means) if cross_track_means else None),
        'worst_abs_cross_track_error_m': (
            max(cross_track_maxima) if cross_track_maxima else None),
        'mean_peak_yaw_rate_deg_s': (
            mean(yaw_rates) if yaw_rates else None),
        'max_yaw_rate_deg_s': max(yaw_rates) if yaw_rates else None,
        'max_alignment_yaw_rate_deg_s': (
            max(alignment_yaw_rates) if alignment_yaw_rates else None),
        'max_alignment_command_yaw_rate_deg_s': (
            max(alignment_command_yaw_rates)
            if alignment_command_yaw_rates else None),
        'mean_in_place_rotation_s': (
            mean(rotation_times) if rotation_times else None),
        'max_in_place_rotation_s': (
            max(rotation_times) if rotation_times else None),
        'max_continuous_rotation_s': (
            max(continuous_rotation_times)
            if continuous_rotation_times else None),
        'max_waypoint_duration_s': (
            max(waypoint_duration_maxima)
            if waypoint_duration_maxima else None),
        'mean_abs_heading_error_deg': (
            mean(heading_error_means) if heading_error_means else None),
        'mean_rms_heading_error_deg': (
            mean(heading_error_rms) if heading_error_rms else None),
        'mean_abs_yaw_rate_tracking_error_deg_s': (
            mean(yaw_rate_error_means) if yaw_rate_error_means else None),
        'mean_rms_yaw_rate_tracking_error_deg_s': (
            mean(yaw_rate_error_rms) if yaw_rate_error_rms else None),
        'mean_integrated_abs_turn_thrust': (
            mean(turn_efforts) if turn_efforts else None),
        'mean_integrated_abs_thruster_command': (
            mean(thruster_efforts) if thruster_efforts else None),
        'mean_turn_thrust_variation_rate': (
            mean(turn_variation_rates) if turn_variation_rates else None),
        'mean_turn_saturation_fraction': (
            mean(saturation_fractions) if saturation_fractions else None),
        'trials': list(trial_reports),
    }


def parse_summary_lines(lines: Iterable[str]) -> Dict[str, Any]:
    """Extract the last complete monitor SUMMARY record from process output."""
    parsed = None
    for line in lines:
        if not line.startswith('SUMMARY '):
            continue
        try:
            candidate = json.loads(line[len('SUMMARY '):])
        except json.JSONDecodeError:
            continue
        if isinstance(candidate, dict):
            parsed = candidate
    if parsed is None:
        raise ValueError('regression monitor did not emit a valid SUMMARY record')
    return parsed


def write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
