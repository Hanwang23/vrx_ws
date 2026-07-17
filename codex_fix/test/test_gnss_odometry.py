import math
import unittest

from codex_usv_controller.core import enu_to_geodetic, geodetic_delta_m
from codex_usv_controller.gnss_odometry import GnssProjector


class GnssProjectorTests(unittest.TestCase):
    def test_projection_round_trip(self):
        origin = (-33.724223, 150.679736)
        latitude, longitude = enu_to_geodetic(*origin, 125.0, -42.0)
        east, north = geodetic_delta_m(*origin, latitude, longitude)
        self.assertAlmostEqual(125.0, east, delta=0.01)
        self.assertAlmostEqual(-42.0, north, delta=0.01)

    def test_velocity_is_smoothed_and_bounded(self):
        projector = GnssProjector(velocity_smoothing=0.5, max_speed=5.0)
        origin = (-33.724223, 150.679736)
        first = projector.update(*origin, 10.0)
        latitude, longitude = enu_to_geodetic(*origin, 2.0, 0.0)
        second = projector.update(latitude, longitude, 11.0)
        self.assertIsNotNone(first)
        self.assertAlmostEqual(1.0, second.velocity_east, delta=0.01)

        jumped_latitude, jumped_longitude = enu_to_geodetic(
            *origin, 100.0, 0.0)
        jumped = projector.update(jumped_latitude, jumped_longitude, 12.0)
        self.assertAlmostEqual(
            second.velocity_east, jumped.velocity_east, delta=0.01)

    def test_invalid_fix_does_not_set_origin(self):
        projector = GnssProjector()
        self.assertIsNone(projector.update(math.nan, 150.0, 1.0))
        self.assertIsNone(projector.origin)


if __name__ == '__main__':
    unittest.main()
