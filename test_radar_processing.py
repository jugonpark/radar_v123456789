import math
import unittest

from radar_processing import (RadarProcessor, Settings, RobotStateManager,
                              validate_points, filter_roi, classify_motion,
                              classify_direction, cluster_points, build_objects,
                              estimate_range_rate)


def points(distance=.6, doppler=-.4, x=0):
    return [dict(x=x+offset, y=distance, z=0, doppler=doppler, snr=20)
            for offset in (0, .02, .04)]


class ProcessingTests(unittest.TestCase):
    def test_static_and_directions(self):
        s = Settings()
        raw = validate_points(points(doppler=0) + points(doppler=.81, x=.2) + points(doppler=-.81, x=-.2))
        roi = filter_roi(raw, s)
        moving = classify_motion(roi, s)
        approaching = classify_direction(moving, s)
        self.assertEqual(sum(p['motion_state'] == 'STATIC' for p in roi), 3)
        self.assertEqual(sum(p['direction_state'] == 'RECEDING' for p in moving), 3)
        self.assertEqual(len(approaching), 3)

    def test_invalid_and_roi(self):
        raw = validate_points([dict(x=float('nan'), y=1, z=0, doppler=0),
                               dict(x=0, y=0, z=0, doppler=0),
                               dict(x=0, y=.5, z=0, doppler=0, snr=None),
                               dict(x=0, y=.5, z=0, doppler=0, snr=20)])
        self.assertEqual(len(raw), 2)
        self.assertEqual(len(filter_roi(raw, Settings())), 1)

    def test_default_range_ends_at_ninety_centimeters(self):
        s = Settings()
        self.assertEqual(s.max_range_m, .90)
        self.assertEqual(len(filter_roi(validate_points(points(.89)), s)), 3)
        self.assertEqual(len(filter_roi(validate_points(points(.91)), s)), 0)

    def test_cluster_and_median(self):
        p = validate_points(points() + [dict(x=.03, y=.6, z=0, doppler=-1.6, snr=20)])
        clusters = cluster_points(p, Settings())
        self.assertEqual(len(clusters), 1)
        self.assertEqual(build_objects(clusters, Settings())[0]['doppler_velocity'], .4)

    def test_range_rate(self):
        self.assertAlmostEqual(estimate_range_rate([(0, .60)], .56, .1), .4)
        self.assertIsNone(estimate_range_rate([(0, .60)], .56, 0))

    def test_temporal_confirmation_and_hysteresis(self):
        p = RadarProcessor()
        first = p.process_frame(1, points(), 0)
        self.assertIsNone(first['target'])
        second = p.process_frame(2, points(.56), .1)
        self.assertFalse(second['objects'][0]['confirmed'])
        third = p.process_frame(3, points(.52), .2)
        self.assertTrue(third['objects'][0]['confirmed'])
        self.assertIsNotNone(third['target'])
        p.process_frame(4, [], .3)
        self.assertIn(1, p.tracks)
        held = p.process_frame(5, [], .4)
        self.assertIsNotNone(held['target'])
        self.assertEqual(held['target']['direction_state'], 'LOST_TEMPORARY')
        self.assertEqual(held['target']['risk'], 'N/A')
        self.assertIn(1, p.tracks)
        p.process_frame(6, [], .5)
        self.assertNotIn(1, p.tracks)

    def test_distance_jump_is_smoothed_and_track_jump_rejected(self):
        p = RadarProcessor()
        p.process_frame(1, points(.60), 0)
        second = p.process_frame(2, points(.56), .1)['objects'][0]
        self.assertGreater(second['distance'], second['raw_distance'])
        self.assertLess(second['distance'], .61)
        jumped = p.process_frame(3, points(.25), .2)['objects'][0]
        self.assertNotEqual(jumped['id'], second['id'])
        self.assertFalse(jumped['confirmed'])

    def test_angle_jump_does_not_inherit_existing_track(self):
        p = RadarProcessor()
        first = p.process_frame(1, points(.60, x=0), 0)['objects'][0]
        second = p.process_frame(2, points(.60, x=.30), .1)['objects'][0]
        self.assertNotEqual(first['id'], second['id'])
        self.assertFalse(second['confirmed'])

    def test_single_frame_noise_and_static_trend(self):
        p = RadarProcessor()
        self.assertIsNone(p.process_frame(1, points(), 0)['target'])
        p.process_frame(2, points(), .1)
        third = p.process_frame(3, points(), .2)
        self.assertEqual(third['objects'][0]['confidence'], 'LOW')
        self.assertIsNone(third['target'])

    def test_high_confidence_and_ttc_risk(self):
        p = RadarProcessor()
        p.process_frame(1, points(.6), 0)
        p.process_frame(2, points(.56), .1)
        third = p.process_frame(3, points(.52), .2)
        obj = third['objects'][0]
        self.assertEqual(obj['confidence'], 'HIGH')
        self.assertEqual(obj['ttc_velocity_source'], 'RANGE_RATE')
        self.assertAlmostEqual(obj['ttc'], obj['distance'] / obj['range_rate_velocity'])
        self.assertEqual(obj['risk'], 'SAFE')

    def test_robot_pause_settle(self):
        p = RadarProcessor()
        p.set_moving(True, 0)
        moving = p.process_frame(1, points(), 0)
        self.assertEqual(moving['processing'], 'PAUSED')
        self.assertIsNone(moving['target'])
        p.set_moving(False, .1)
        self.assertEqual(p.process_frame(2, points(), .2)['robot_state'], 'SETTLING')
        self.assertEqual(p.process_frame(3, points(), .4)['robot_state'], 'MONITORING')

    def test_risk_boundaries(self):
        from radar_processing import calculate_risk
        s = Settings()
        for ttc, expected in ((3.0, 'WATCH'), (2.0, 'WARNING'), (1.2, 'AVOID'), (.6, 'STOP'), (3.1, 'SAFE')):
            self.assertEqual(calculate_risk(dict(distance=.8, ttc=ttc), s), expected)


if __name__ == '__main__':
    unittest.main()
