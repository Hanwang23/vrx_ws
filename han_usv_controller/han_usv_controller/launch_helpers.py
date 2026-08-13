"""Pure helpers shared by the ROS launch descriptions."""

import fcntl
import hashlib
import os
from typing import TextIO, Tuple


class SimulationAlreadyRunningError(RuntimeError):
    """Raised before launch when this workspace already owns a simulation."""


def launch_flag(value: object) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def resolve_simulation_world(
    controller_share: str,
    requested_world: str,
    timed_competition: object,
) -> str:
    if requested_world == 'wayfinding_task' and not launch_flag(
        timed_competition
    ):
        return os.path.join(
            controller_share, 'worlds', 'wayfinding_task.sdf')
    return requested_world


def resolve_gz_world_name(requested_world: str, override: str = '') -> str:
    if override.strip():
        return override.strip()
    world_without_extension, _extension = os.path.splitext(requested_world)
    return os.path.basename(world_without_extension)


def simulation_lock_path(controller_share: str) -> str:
    """Build a stable, workspace-specific lock path outside the install tree."""
    workspace_key = os.path.realpath(controller_share).encode('utf-8')
    digest = hashlib.sha256(workspace_key).hexdigest()[:12]
    return os.path.join('/tmp', f'han_usv_simulation_{digest}.lock')


def acquire_simulation_lock(lock_path: str) -> TextIO:
    """Acquire the launch lock without waiting, leaving ownership to the FD."""
    stream = open(lock_path, 'a+', encoding='utf-8')
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        stream.seek(0)
        owner = stream.read().strip() or 'unknown PID'
        stream.close()
        raise SimulationAlreadyRunningError(
            'A han_usv_controller simulation is already running '
            f'(owner {owner}, lock {lock_path}). Stop the existing launch '
            'with Ctrl+C before starting another one.'
        ) from error
    stream.seek(0)
    stream.truncate()
    stream.write(f'PID {os.getpid()}')
    stream.flush()
    return stream


def release_simulation_lock(stream: TextIO) -> None:
    """Release a lock explicitly; process exit also closes it automatically."""
    if stream.closed:
        return
    fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
    stream.close()


def find_gazebo_sim_processes(
    proc_root: str = '/proc',
) -> Tuple[Tuple[int, str], ...]:
    """Find already-running Gazebo Sim servers before launching another one."""
    matches = []
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return ()
    for entry in entries:
        if not entry.isdigit():
            continue
        try:
            with open(
                os.path.join(proc_root, entry, 'cmdline'), 'rb'
            ) as stream:
                arguments = [
                    part.decode('utf-8', errors='replace')
                    for part in stream.read().split(b'\0')
                    if part
                ]
        except OSError:
            continue
        direct_gz = (
            len(arguments) >= 2
            and os.path.basename(arguments[0]) == 'gz'
            and arguments[1] == 'sim'
        )
        ruby_gz_wrapper = (
            len(arguments) >= 3
            and os.path.basename(arguments[0]).startswith('ruby')
            and os.path.basename(arguments[1]) == 'gz'
            and arguments[2] == 'sim'
        )
        if direct_gz or ruby_gz_wrapper:
            matches.append((int(entry), ' '.join(arguments)))
    return tuple(sorted(matches))


def reject_running_gazebo(proc_root: str = '/proc') -> None:
    """Reject stale or externally launched Gazebo servers without a lock."""
    conflicts = find_gazebo_sim_processes(proc_root)
    if not conflicts:
        return
    details = '; '.join(
        f'PID {pid}: {command}' for pid, command in conflicts)
    raise SimulationAlreadyRunningError(
        'An existing Gazebo Sim server was detected before launch '
        f'({details}). Stop that simulation with Ctrl+C before starting '
        'han_usv_controller.'
    )
