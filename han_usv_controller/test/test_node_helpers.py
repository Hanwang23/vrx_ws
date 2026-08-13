import math
from types import SimpleNamespace
import unittest

from han_usv_controller.core import validated_quaternion_yaw
from han_usv_controller.node import AutonomousUSVNode


class Resettable:
    def __init__(self):
        self.calls = 0

    def reset(self):
        self.calls += 1


class NodeHelperTests(unittest.TestCase):
    def test_validated_quaternion_yaw_normalizes_input(self):
        yaw = validated_quaternion_yaw(0.0, 0.0, 0.5, 0.5)
        self.assertAlmostEqual(yaw, math.pi / 2.0)

    def test_validated_quaternion_yaw_rejects_zero_and_nan(self):
        self.assertIsNone(validated_quaternion_yaw(0.0, 0.0, 0.0, 0.0))
        self.assertIsNone(
            validated_quaternion_yaw(0.0, 0.0, math.nan, 1.0))

    def test_task_runtime_reset_clears_estimation_and_perception(self):
        runtime = SimpleNamespace(
            estimator=Resettable(),
            speed_estimator=Resettable(),
            occupancy_grid=Resettable(),
            latitude=1.0,
            longitude=2.0,
            yaw=3.0,
            yaw_rate=4.0,
            yaw_rate_valid=True,
            speed=5.0,
            last_imu_sample_time=1.0,
            last_imu_yaw=1.0,
            last_gps_time=1.0,
            last_imu_time=1.0,
            last_scan_time=1.0,
            last_cloud_time=1.0,
            last_control_time=1.0,
            laser_ranges=(1.0,),
            obstacle_points=((1.0, 0.0),),
            cloud_tracks=[(1.0, 2.0, 3.0, 4)],
            buoy_candidate_count=3,
            dynamic_tracker=Resettable(),
            current_encounter=object(),
            colregs_risk_count=2,
            pose_history=[(1.0, 2.0, 3.0, 4.0)],
            latest_occupancy_snapshot=object(),
            last_map_publish_time=1.0,
            map_known_cells=10,
            map_occupied_cells=2,
            ekf_enabled=True,
            estimator_healthy=True,
            estimator_position_std=1.0,
            estimator_velocity_std=1.0,
            estimator_yaw_std=1.0,
        )

        AutonomousUSVNode._reset_runtime_state(runtime)

        self.assertEqual(runtime.estimator.calls, 1)
        self.assertEqual(runtime.speed_estimator.calls, 1)
        self.assertEqual(runtime.occupancy_grid.calls, 1)
        self.assertIsNone(runtime.latitude)
        self.assertIsNone(runtime.yaw)
        self.assertFalse(runtime.estimator_healthy)
        self.assertEqual(runtime.laser_ranges, ())
        self.assertEqual(runtime.map_known_cells, 0)
        self.assertEqual(runtime.buoy_candidate_count, 0)
        self.assertEqual(runtime.dynamic_tracker.calls, 1)
        self.assertIsNone(runtime.current_encounter)
        self.assertEqual(runtime.colregs_risk_count, 0)


if __name__ == '__main__':
    unittest.main()
