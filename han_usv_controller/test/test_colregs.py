import math
import unittest

from han_usv_controller.colregs import (
    DynamicTargetTracker,
    DynamicTrack,
    EncounterAssessment,
    assess_encounter,
    is_confirmed_moving,
    select_most_urgent,
)


class ColregsTests(unittest.TestCase):
    def test_tracker_estimates_moving_target(self):
        tracker = DynamicTargetTracker(position_gain=1.0, velocity_gain=1.0)
        for index in range(5):
            tracker.update([(10.0 - index, 0.0)], float(index))
        track = tracker.active_tracks(4.0)[0]
        self.assertGreaterEqual(track.hits, 4)
        self.assertAlmostEqual(-1.0, track.velocity_east, delta=0.05)

    def test_tracker_preserves_two_interleaved_target_publishers(self):
        tracker = DynamicTargetTracker(
            match_distance=4.0,
            timeout=4.0,
            position_gain=1.0,
            velocity_gain=1.0,
        )
        for index in range(6):
            tracker.update([(10.0 - index, 0.0)], float(index))
            tracker.update([(30.0, 10.0 - 0.5 * index)], index + 0.01)
        tracks = sorted(
            tracker.active_tracks(5.01), key=lambda track: track.east)
        self.assertEqual(2, len(tracks))
        self.assertAlmostEqual(-1.0, tracks[0].velocity_east, delta=0.05)
        self.assertAlmostEqual(-0.5, tracks[1].velocity_north, delta=0.05)

    def test_head_on_encounter_commands_starboard(self):
        target = DynamicTrack(
            1, east=30.0, north=0.0,
            velocity_east=-1.0, velocity_north=0.0, hits=10,
            timestamp=5.0, first_east=40.0, first_north=0.0,
            first_timestamp=0.0, motion_consistency=10,
            covariance_m2=0.5)
        result = assess_encounter(
            (0.0, 0.0), (1.0, 0.0), 0.0, target)
        self.assertTrue(result.risk)
        self.assertEqual('head_on', result.encounter)
        self.assertLess(result.heading_bias, 0.0)
        self.assertLess(result.speed_scale, 1.0)

    def test_static_buoy_never_triggers_colregs(self):
        target = DynamicTrack(
            1, east=10.0, north=-2.0,
            velocity_east=0.0, velocity_north=0.0, hits=20)
        result = assess_encounter(
            (0.0, 0.0), (1.0, 0.0), 0.0, target)
        self.assertFalse(result.risk)

    def test_urgent_selection_uses_earliest_tcpa(self):
        first = assess_encounter(
            (0.0, 0.0), (1.0, 0.0), 0.0,
            DynamicTrack(
                1, 20.0, 0.0, -1.0, 0.0, hits=10,
                timestamp=5.0, first_east=30.0, first_north=0.0,
                first_timestamp=0.0, motion_consistency=10,
                covariance_m2=0.5))
        second = assess_encounter(
            (0.0, 0.0), (1.0, 0.0), 0.0,
            DynamicTrack(
                2, 40.0, 0.0, -1.0, 0.0, hits=10,
                timestamp=5.0, first_east=50.0, first_north=0.0,
                first_timestamp=0.0, motion_consistency=10,
                covariance_m2=0.5))
        self.assertEqual(1, select_most_urgent((second, first)).track_id)

    def test_own_vessel_overtaking_target_must_keep_clear(self):
        target = DynamicTrack(
            1, east=20.0, north=0.0,
            velocity_east=1.0, velocity_north=0.0, hits=10,
            timestamp=5.0, first_east=15.0, first_north=0.0,
            first_timestamp=0.0, motion_consistency=10,
            covariance_m2=0.5)
        result = assess_encounter(
            (0.0, 0.0), (2.0, 0.0), 0.0, target)
        self.assertTrue(result.risk)
        self.assertEqual('overtaking', result.encounter)
        self.assertEqual('keep_clear_starboard', result.action)

    def test_target_overtaking_from_astern_is_stand_on(self):
        target = DynamicTrack(
            1, east=-20.0, north=0.0,
            velocity_east=2.0, velocity_north=0.0, hits=10,
            timestamp=5.0, first_east=-30.0, first_north=0.0,
            first_timestamp=0.0, motion_consistency=10,
            covariance_m2=0.5)
        result = assess_encounter(
            (0.0, 0.0), (1.0, 0.0), 0.0, target)
        self.assertTrue(result.risk)
        self.assertEqual('being_overtaken', result.encounter)
        self.assertEqual('stand_on_monitor', result.action)

    def test_give_way_target_has_priority_over_earlier_stand_on_target(self):
        stand_on = EncounterAssessment(
            1, 'crossing_port', 'stand_on_monitor', 10.0, 0.5,
            3.0, 2.0, True)
        give_way = EncounterAssessment(
            2, 'crossing_starboard', 'give_way_starboard', 20.0, -0.5,
            6.0, 3.0, True)
        self.assertEqual(2, select_most_urgent((stand_on, give_way)).track_id)

    def test_map_mask_confirmation_uses_same_motion_gate(self):
        target = DynamicTrack(
            1, east=10.0, north=0.0,
            velocity_east=1.0, velocity_north=0.0, hits=10,
            timestamp=5.0, first_east=5.0, first_north=0.0,
            first_timestamp=0.0, motion_consistency=10,
            covariance_m2=0.5)
        self.assertTrue(is_confirmed_moving(target, 0.5, 8))
        target.motion_consistency = 0
        self.assertFalse(is_confirmed_moving(target, 0.5, 8))


if __name__ == '__main__':
    unittest.main()
