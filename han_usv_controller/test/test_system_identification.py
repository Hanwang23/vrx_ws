import math
from pathlib import Path
import tempfile
import unittest

from han_usv_controller.model_control import gate_model_control
from han_usv_controller.system_identification import (
    IdentificationLogger,
    IdentificationSample,
    fit_three_dof_model,
    load_samples,
)


class SystemIdentificationTests(unittest.TestCase):
    def synthetic_samples(self):
        samples = []
        surge = 0.0
        yaw_rate = 0.0
        dt = 0.1
        for index in range(500):
            average = 250.0 if (index // 50) % 2 == 0 else 450.0
            differential = -80.0 if (index // 35) % 2 == 0 else 80.0
            left = average - differential
            right = average + differential
            surge += dt * (-0.6 * surge + 0.002 * average)
            yaw_rate += dt * (-1.0 * yaw_rate + 0.001 * differential)
            samples.append(IdentificationSample(
                index * dt, 0.0, 0.0, 0.0, surge, 0.0, yaw_rate,
                left, right, 'navigating'))
        return samples

    def test_fitted_excited_model_passes_readiness(self):
        result = fit_three_dof_model(self.synthetic_samples())
        self.assertTrue(result['nmpc_ready'])
        self.assertAlmostEqual(
            -0.6, result['surge']['state_coefficient'], delta=0.08)

    def test_unexcited_data_fails_safe_to_ilos(self):
        result = fit_three_dof_model(self.synthetic_samples()[:10])
        status = gate_model_control('nmpc', result)
        self.assertFalse(status.nmpc_ready)
        self.assertEqual('ilos_pid', status.active_backend)

    def test_logger_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'samples.csv'
            logger = IdentificationLogger(str(path), minimum_period=0.1)
            sample = self.synthetic_samples()[0]
            self.assertTrue(logger.append(sample))
            self.assertEqual([sample], load_samples(str(path)))


if __name__ == '__main__':
    unittest.main()
