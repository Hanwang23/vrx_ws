import math
import unittest

from han_usv_controller.estimator import EstimatorConfig, PlanarEKF


class PlanarEKFTests(unittest.TestCase):
    def initialize(self, estimator):
        estimator.update_gps(30.0, -90.0, 0.0, (0.25, 0.25))
        estimator.update_imu(0.0, 0.0, 0.0, 0.01, 0.01)

    def test_requires_position_and_heading(self):
        estimator = PlanarEKF()
        estimator.update_gps(30.0, -90.0, 0.0)
        self.assertIsNone(estimator.estimate())
        estimator.update_imu(0.0, 0.0, 0.0)
        self.assertIsNotNone(estimator.estimate())

    def test_constant_east_motion_estimates_forward_speed(self):
        estimator = PlanarEKF()
        self.initialize(estimator)
        longitude_per_meter = math.degrees(
            1.0 / (6371000.0 * math.cos(math.radians(30.0))))
        for second in range(1, 8):
            estimator.update_gps(
                30.0, -90.0 + second * longitude_per_meter,
                float(second), (0.04, 0.04))
            estimator.update_imu(0.0, 0.0, float(second), 0.001, 0.001)
        estimate = estimator.estimate()
        self.assertTrue(estimate.healthy)
        self.assertAlmostEqual(estimate.forward_speed, 1.0, delta=0.2)
        self.assertAlmostEqual(estimate.east, 7.0, delta=0.5)

    def test_heading_update_crosses_pi_without_large_jump(self):
        estimator = PlanarEKF()
        estimator.update_gps(30.0, -90.0, 0.0)
        estimator.update_imu(math.radians(179.0), 0.0, 0.0, 0.001, 0.001)
        estimator.update_imu(math.radians(-179.0), 0.0, 0.1, 0.001, 0.001)
        estimate = estimator.estimate()
        self.assertLess(
            abs(math.degrees(abs(estimate.yaw) - math.pi)), 2.0)

    def test_large_gps_outlier_is_rejected(self):
        estimator = PlanarEKF(EstimatorConfig(innovation_gate_sigma=4.0))
        self.initialize(estimator)
        accepted = estimator.update_gps(31.0, -89.0, 1.0, (0.25, 0.25))
        estimate = estimator.estimate()
        self.assertFalse(accepted)
        self.assertLess(math.hypot(estimate.east, estimate.north), 10.0)
        self.assertEqual(estimate.rejected_measurements, 1)

    def test_prediction_covariance_remains_symmetric_and_positive(self):
        estimator = PlanarEKF()
        self.initialize(estimator)
        for index in range(1, 20):
            estimator.predict(index * 0.1)
        for row in range(estimator.STATE_SIZE):
            self.assertGreater(estimator.covariance[row][row], 0.0)
            for column in range(estimator.STATE_SIZE):
                self.assertAlmostEqual(
                    estimator.covariance[row][column],
                    estimator.covariance[column][row], places=9)


if __name__ == '__main__':
    unittest.main()
