"""Run isolated, repeated VRX Wayfinding evaluations."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Iterable, List, Optional, Sequence, Tuple

from .evaluation import (
    EvaluationThresholds,
    aggregate_trials,
    analyze_trial,
    parse_summary_lines,
    write_json,
)
from .random_course import layout_manifest
from .multi_waypoint_course import course_manifest


STALE_PROCESS_PATTERNS = (
    'ros2 launch han_usv_controller',
    'ros2 run han_usv_controller run_evaluation',
    'install/lib/han_usv_controller/run_evaluation',
    'install/lib/han_usv_controller/regression_monitor',
    'install/lib/han_usv_controller/autonomous_usv',
    'install/lib/han_usv_controller/moving_target',
    'install/lib/han_usv_controller/gnss_odometry_adapter',
    '/robot_localization/ekf_node',
    'gz sim',
    'monitor_sim.py',
    'ros_gz_bridge/parameter_bridge',
)

ProcessRecord = Tuple[int, str]


def _terminate_group(process: subprocess.Popen, timeout: float = 15.0) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)


def _ancestor_pids(pid: int) -> set:
    ancestors = set()
    current = int(pid)
    while current > 1 and current not in ancestors:
        ancestors.add(current)
        try:
            status = Path(f'/proc/{current}/status').read_text(encoding='utf-8')
        except (OSError, UnicodeError):
            break
        parent_line = next(
            (line for line in status.splitlines() if line.startswith('PPid:')),
            None,
        )
        if parent_line is None:
            break
        current = int(parent_line.split(':', 1)[1].strip())
    return ancestors


def _matching_processes(
    lines: Iterable[str],
    ignored_pids: Sequence[int] = (),
) -> List[ProcessRecord]:
    ignored = set(ignored_pids)
    matches: List[ProcessRecord] = []
    for line in lines:
        fields = line.strip().split(maxsplit=1)
        if len(fields) != 2:
            continue
        try:
            pid = int(fields[0])
        except ValueError:
            continue
        command = fields[1]
        if pid in ignored:
            continue
        if any(pattern in command for pattern in STALE_PROCESS_PATTERNS):
            matches.append((pid, command))
    return matches


def _find_stale_processes() -> List[ProcessRecord]:
    listing = subprocess.run(
        ['ps', '-eo', 'pid=,args='], text=True, capture_output=True, check=True)
    return _matching_processes(
        listing.stdout.splitlines(), _ancestor_pids(os.getpid()))


def _clean_stale_processes(force: bool = False) -> None:
    victims = _find_stale_processes()
    if victims and not force:
        details = '\n'.join(
            f'  PID {pid}: {command}' for pid, command in victims[:8])
        raise RuntimeError(
            'another ROS/Gazebo evaluation process is already running; stop it '
            'before starting a new evaluation, or explicitly pass '
            f'--force-clean-stale:\n{details}')
    victim_pids = [pid for pid, _ in victims]
    for sig in (signal.SIGINT, signal.SIGTERM):
        for pid in victim_pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        if victim_pids:
            time.sleep(2.0)
        victim_pids = [
            pid for pid in victim_pids if Path(f'/proc/{pid}').exists()]
        if not victim_pids:
            break
    for pid in victim_pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    if force and victims:
        subprocess.run(
            ['ros2', 'daemon', 'stop'], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, check=False)
        time.sleep(1.0)


def _run_trial(
    index: int,
    output_dir: Path,
    timeout: float,
    startup_delay: float,
    thresholds: EvaluationThresholds,
    launch_file: str,
    scenario_seed: Optional[int] = None,
    completion_grace: float = 2.0,
) -> dict:
    trial_dir = output_dir / f'trial_{index:02d}'
    trial_dir.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment['ROS_LOG_DIR'] = str(trial_dir / 'ros_logs')
    launch_log_path = trial_dir / 'launch.log'
    monitor_log_path = trial_dir / 'monitor.log'
    launch_command = [
        'ros2', 'launch', 'han_usv_controller', launch_file,
        'headless:=True', 'rviz:=False', 'timed_competition:=False',
    ]
    manifest = None
    if scenario_seed is not None:
        manifest = layout_manifest(scenario_seed)
        launch_command.append(f'scenario_seed:={scenario_seed}')
    elif launch_file == 'multi_waypoint_course.launch.py':
        manifest = course_manifest()
    if manifest is not None:
        write_json(trial_dir / 'scenario.json', manifest)
    monitor_command = [
        'ros2', 'run', 'han_usv_controller', 'regression_monitor',
        '--timeout', str(timeout), '--live-period', '5.0',
        '--completion-grace', str(completion_grace),
    ]
    launch_process: Optional[subprocess.Popen] = None
    monitor_process: Optional[subprocess.Popen] = None
    started = time.monotonic()
    try:
        with launch_log_path.open('w', encoding='utf-8') as launch_log:
            launch_process = subprocess.Popen(
                launch_command, stdout=launch_log, stderr=subprocess.STDOUT,
                env=environment, start_new_session=True, text=True)
            time.sleep(startup_delay)
            if launch_process.poll() is not None:
                raise RuntimeError(
                    f'simulation launch exited early with {launch_process.returncode}')
            with monitor_log_path.open('w', encoding='utf-8') as monitor_log:
                monitor_process = subprocess.Popen(
                    monitor_command, stdout=monitor_log,
                    stderr=subprocess.STDOUT, env=environment,
                    start_new_session=True, text=True)
                monitor_process.wait(timeout=timeout + 30.0)
        summary = parse_summary_lines(
            monitor_log_path.read_text(encoding='utf-8').splitlines())
        verdict = analyze_trial(summary, thresholds)
        report = {
            'trial': index,
            'scenario_seed': scenario_seed,
            'scenario': manifest,
            'launch_command': launch_command,
            'monitor_return_code': monitor_process.returncode,
            'summary': summary,
            'verdict': verdict,
        }
    except Exception as error:  # Keep later trials and diagnostics available.
        report = {
            'trial': index,
            'scenario_seed': scenario_seed,
            'scenario': manifest,
            'summary': {
                'completed': False,
                'elapsed_wall_s': time.monotonic() - started,
                'status_samples': 0,
            },
            'verdict': {'passed': False, 'issues': [str(error)], 'metrics': {}},
        }
    finally:
        if monitor_process is not None:
            _terminate_group(monitor_process)
        if launch_process is not None:
            _terminate_group(launch_process)
    write_json(trial_dir / 'result.json', report)
    return report


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument(
        '--max-wall-time', type=float, default=None,
        help='Pass/fail wall-time limit; defaults to the watchdog timeout')
    parser.add_argument('--startup-delay', type=float, default=8.0)
    parser.add_argument('--output-root', default='han_usv_controller/evaluation')
    parser.add_argument('--no-clean-stale', action='store_true')
    parser.add_argument(
        '--force-clean-stale', action='store_true',
        help='Explicitly stop all matching ROS/Gazebo processes before a trial')
    parser.add_argument('--max-mean-error', type=float, default=2.5)
    parser.add_argument('--max-waypoint-error', type=float, default=3.0)
    parser.add_argument(
        '--launch-file',
        choices=(
            'simulation.launch.py',
            'buoy_course.launch.py',
            'lattice_stress.launch.py',
            'random_buoy_course.launch.py',
            'multi_waypoint_course.launch.py',
            'colregs_learning.launch.py',
        ),
        default='simulation.launch.py')
    parser.add_argument(
        '--base-seed', type=int, default=1000,
        help='First deterministic seed for random_buoy_course.launch.py')
    parser.add_argument('--min-lattice-expanded-states', type=int, default=0)
    parser.add_argument('--max-lattice-fallback-samples', type=int, default=0)
    parser.add_argument('--min-colregs-active-samples', type=int, default=0)
    parser.add_argument('--min-dynamic-track-count', type=int, default=0)
    parser.add_argument('--expected-waypoint-count', type=int, default=0)
    parser.add_argument('--max-alignment-yaw-rate', type=float, default=0.0)
    parser.add_argument(
        '--max-alignment-command-yaw-rate', type=float, default=0.0)
    parser.add_argument('--max-continuous-rotation', type=float, default=0.0)
    parser.add_argument('--max-waypoint-duration', type=float, default=0.0)
    parser.add_argument('--max-estimator-fallback-count', type=int, default=-1)
    parser.add_argument('--completion-grace', type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.trials < 1:
        parser.error('--trials must be at least 1')
    if args.no_clean_stale and args.force_clean_stale:
        parser.error('--no-clean-stale and --force-clean-stale are mutually exclusive')

    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output_dir = Path(args.output_root).resolve() / timestamp
    output_dir.mkdir(parents=True, exist_ok=False)
    expected_waypoint_count = args.expected_waypoint_count
    if (
        expected_waypoint_count == 0
        and args.launch_file == 'multi_waypoint_course.launch.py'
    ):
        expected_waypoint_count = 8
    thresholds = EvaluationThresholds(
        max_wall_time_s=(
            args.max_wall_time
            if args.max_wall_time is not None else args.timeout),
        max_mean_error_m=args.max_mean_error,
        max_waypoint_error_m=args.max_waypoint_error,
        min_lattice_expanded_states=args.min_lattice_expanded_states,
        max_lattice_fallback_samples=args.max_lattice_fallback_samples,
        min_colregs_active_samples=args.min_colregs_active_samples,
        min_dynamic_track_count=args.min_dynamic_track_count,
        expected_waypoint_count=expected_waypoint_count,
        max_alignment_yaw_rate_deg_s=args.max_alignment_yaw_rate,
        max_alignment_command_yaw_rate_deg_s=(
            args.max_alignment_command_yaw_rate),
        max_continuous_rotation_s=args.max_continuous_rotation,
        max_waypoint_duration_s=args.max_waypoint_duration,
        max_estimator_fallback_count=args.max_estimator_fallback_count,
    )
    reports = []
    for index in range(1, args.trials + 1):
        if not args.no_clean_stale:
            _clean_stale_processes(force=args.force_clean_stale)
        try:
            report = _run_trial(
                index,
                output_dir,
                args.timeout,
                args.startup_delay,
                thresholds,
                args.launch_file,
                (
                    args.base_seed + index - 1
                    if args.launch_file == 'random_buoy_course.launch.py'
                    else None
                ),
                args.completion_grace,
            )
        except KeyboardInterrupt:
            print('Evaluation interrupted; child processes were cleaned up.')
            return 130
        reports.append(report)
        verdict = report['verdict']
        print(
            f'TRIAL {index}/{args.trials}: '
            f'{"PASS" if verdict["passed"] else "FAIL"} '
            + ('; '.join(verdict['issues']) or 'all thresholds satisfied'),
            flush=True,
        )
    aggregate = aggregate_trials(reports, thresholds)
    write_json(output_dir / 'aggregate.json', aggregate)
    latest = Path(args.output_root).resolve() / 'latest.json'
    write_json(latest, aggregate)
    print('AGGREGATE ' + json.dumps(aggregate, separators=(',', ':')))
    print(f'Artifacts: {output_dir}')
    return 0 if aggregate['passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
