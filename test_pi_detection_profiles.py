import unittest

from radar_processing import RadarProcessor, filter_roi, validate_points
from raspberry_pi.profiles import settings_for


def point(distance, doppler=-.81, snr=12, x=0):
    return dict(x=x, y=distance, z=0, doppler=doppler, snr=snr)


class DetectionProfileTests(unittest.TestCase):
    def test_approach_profile_modes_preserve_legacy_ranges(self):
        self.assertEqual(settings_for("STRICT").approach_path_mode,"OFF")
        for profile in ("BALANCED","DIAGNOSTIC"):
            settings=settings_for(profile)
            self.assertEqual(settings.approach_path_mode,"OBSERVE")
            self.assertEqual(settings.max_range_m,1.5)
            self.assertEqual(settings.approach_observe_max_distance_m,1.2)
            self.assertEqual(settings.approach_avoid_max_distance_m,.8)

    def test_single_point_is_candidate_not_instant_target(self):
        processor = RadarProcessor(settings_for("BALANCED"))
        first = processor.process_frame(1, [point(.8)], 0)
        self.assertEqual(first["counts"]["single_point_clusters"], 1)
        self.assertEqual(first["counts"]["tentative"], 1)
        self.assertEqual(first["objects"][0]["pending_reason"], "WAITING_PERSISTENCE")
        self.assertIsNone(first["target"])

    def test_two_of_three_persists_with_one_miss(self):
        processor = RadarProcessor(settings_for("BALANCED"))
        processor.process_frame(1, [point(.8)], 0)
        processor.process_frame(2, [], .1)
        third = processor.process_frame(3, [point(.65)], .2)
        self.assertTrue(third["objects"][0]["confirmed"])

    def test_two_frame_fast_approach_needs_both_doppler_and_range_trend(self):
        processor = RadarProcessor(settings_for("BALANCED"))
        processor.process_frame(1, [point(.7)], 0)
        second = processor.process_frame(2, [point(.5)], .1)
        self.assertEqual(second["objects"][0]["confidence"], "HIGH")
        self.assertIsNotNone(second["target"])
        static_range = RadarProcessor(settings_for("BALANCED"))
        static_range.process_frame(1, [point(.7)], 0)
        unchanged = static_range.process_frame(2, [point(.7)], .1)
        self.assertIsNone(unchanged["target"])

    def test_mixed_sign_cluster_majority_approaches(self):
        processor = RadarProcessor(settings_for("BALANCED"))
        points = [point(.65, d, x=x) for x, d in ((0,-.81),(.02,-.81),(.04,-.81),(.06,.81))]
        result = processor.process_frame(1, points, 0)
        self.assertEqual(len(result["objects"]), 1)
        self.assertEqual(result["objects"][0]["direction_state"], "APPROACHING")
        self.assertEqual(result["objects"][0]["approaching_point_ratio"], .75)

    def test_detection_and_threat_ranges_are_separate(self):
        processor = RadarProcessor(settings_for("BALANCED"))
        for i, d in enumerate((1.4,1.2,1.0), 1):
            result = processor.process_frame(i, [point(d)], i*.1)
        self.assertTrue(result["objects"])
        self.assertIsNone(result["target"])
        self.assertEqual(result["objects"][0]["risk"], "SAFE")

    def test_snr_and_missing_side_info(self):
        low = validate_points([point(.6, snr=7), point(.7, snr=None)])
        self.assertEqual(len(filter_roi(low, settings_for("STRICT"))), 0)
        self.assertEqual(len(filter_roi(low, settings_for("BALANCED"))), 0)
        self.assertEqual(len(filter_roi(low, settings_for("DIAGNOSTIC"))), 2)

    def test_receding_and_static_do_not_target(self):
        for doppler in (.81, 0):
            processor = RadarProcessor(settings_for("BALANCED"))
            for i, d in enumerate((.6,.65,.7), 1):
                result = processor.process_frame(i, [point(d, doppler)], i*.1)
            self.assertIsNone(result["target"])

    def test_diagnostic_profile_never_selects_target(self):
        processor = RadarProcessor(settings_for("DIAGNOSTIC"))
        for i, d in enumerate((.6,.5,.4), 1):
            result = processor.process_frame(i, [point(d, snr=None)], i*.1)
        self.assertIsNone(result["target"])
        self.assertGreater(result["counts"]["snr_missing"], 0)


if __name__ == "__main__":
    unittest.main()
