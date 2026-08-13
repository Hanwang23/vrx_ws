from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

from han_usv_controller.launch_helpers import (
    acquire_simulation_lock,
    find_gazebo_sim_processes,
    launch_flag,
    reject_running_gazebo,
    release_simulation_lock,
    resolve_gz_world_name,
    resolve_simulation_world,
    simulation_lock_path,
)


class LearningWorldTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).parents[1]

    def test_learning_world_keeps_wayfinding_service_name(self):
        world = ET.parse(
            self.root / 'worlds' / 'wayfinding_task.sdf'
        ).getroot().find('./world')
        self.assertIsNotNone(world)
        self.assertEqual('wayfinding_task', world.attrib['name'])

    def test_learning_world_has_effectively_unlimited_duration(self):
        root = ET.parse(
            self.root / 'worlds' / 'wayfinding_task.sdf'
        ).getroot()
        duration = float(root.findtext(
            ".//plugin[@name='vrx::WayfindingScoringPlugin']"
            '/running_state_duration'
        ))
        self.assertGreaterEqual(duration, 100.0 * 365.0 * 24.0 * 3600.0)

    def test_learning_world_resolution_is_limited_to_untimed_wayfinding(self):
        share = '/tmp/han_usv_controller'
        expected = '/tmp/han_usv_controller/worlds/wayfinding_task.sdf'
        self.assertEqual(
            expected,
            resolve_simulation_world(share, 'wayfinding_task', 'False'),
        )
        self.assertEqual(
            'wayfinding_task',
            resolve_simulation_world(share, 'wayfinding_task', 'True'),
        )
        self.assertEqual(
            'stationkeeping_task',
            resolve_simulation_world(share, 'stationkeeping_task', 'False'),
        )

    def test_launch_boolean_accepts_ros_conventions(self):
        for value in ('1', 'true', 'TRUE', 'yes', 'on'):
            self.assertTrue(launch_flag(value))
        for value in ('0', 'false', 'no', 'off', ''):
            self.assertFalse(launch_flag(value))

    def test_gazebo_world_name_is_derived_but_can_be_overridden(self):
        self.assertEqual(
            'wayfinding_task',
            resolve_gz_world_name('/tmp/worlds/wayfinding_task.sdf'),
        )
        self.assertEqual(
            'stationkeeping_task',
            resolve_gz_world_name('stationkeeping_task'),
        )
        self.assertEqual(
            'internal_name',
            resolve_gz_world_name('/tmp/custom.sdf', 'internal_name'),
        )

    def test_simulation_lock_is_workspace_specific(self):
        first = simulation_lock_path('/tmp/workspace-a/install/share/package')
        second = simulation_lock_path('/tmp/workspace-b/install/share/package')
        self.assertNotEqual(first, second)
        self.assertTrue(first.startswith('/tmp/han_usv_simulation_'))

    def test_ruby_wrapped_gazebo_is_detected(self):
        proc_root = self.root / 'proc'
        process = proc_root / '4321'
        process.mkdir(parents=True)
        (process / 'cmdline').write_bytes(
            b'ruby\0/usr/bin/gz\0sim\0-v\04\0-r\0-s\0world.sdf\0')
        matches = find_gazebo_sim_processes(str(proc_root))
        self.assertEqual(1, len(matches))
        self.assertEqual(4321, matches[0][0])

    def test_simulation_lock_rejects_overlap_and_releases(self):
        lock_path = simulation_lock_path(str(self.root))
        lock = acquire_simulation_lock(lock_path)
        script = (
            'from han_usv_controller.launch_helpers import '
            'acquire_simulation_lock; '
            f'acquire_simulation_lock({lock_path!r})'
        )
        try:
            blocked = subprocess.run(
                [sys.executable, '-c', script],
                cwd=self.root,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertNotEqual(0, blocked.returncode)
            self.assertIn('already running', blocked.stderr)
        finally:
            release_simulation_lock(lock)

        released = subprocess.run(
            [sys.executable, '-c', script],
            cwd=self.root,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(0, released.returncode, released.stderr)

    def test_simulation_launch_connects_the_exclusive_lock(self):
        source = (self.root / 'launch' / 'simulation.launch.py').read_text(
            encoding='utf-8')
        self.assertIn('acquire_simulation_lock(', source)
        self.assertIn('reject_running_gazebo()', source)
        self.assertIn('OnShutdown(on_shutdown=_release_launch_lock)', source)

    def test_gazebo_preflight_detects_only_server_processes(self):
        with tempfile.TemporaryDirectory() as proc_root:
            gazebo = Path(proc_root) / '321'
            gazebo.mkdir()
            (gazebo / 'cmdline').write_bytes(
                b'gz\0sim\0-r\0example.sdf\0')
            unrelated = Path(proc_root) / '654'
            unrelated.mkdir()
            (unrelated / 'cmdline').write_bytes(
                b'python3\0some_node.py\0')

            self.assertEqual(
                ((321, 'gz sim -r example.sdf'),),
                find_gazebo_sim_processes(proc_root),
            )
            with self.assertRaisesRegex(RuntimeError, 'PID 321'):
                reject_running_gazebo(proc_root)


if __name__ == '__main__':
    unittest.main()
