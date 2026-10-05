import unittest
from dataclasses import replace
from radar_processing import RadarProcessor
from raspberry_pi.profiles import settings_for
from test_radar_fast_path import cloud


def run(distances, doppler=(-.81,.81), **overrides):
    processor=RadarProcessor(replace(settings_for("BALANCED"), **overrides))
    return [processor.process_frame(i+1,[] if d is None else cloud(d,doppler), i*.1) for i,d in enumerate(distances)]


class ApproachDecisionTests(unittest.TestCase):
    def primary(self,distances,**settings):
        return run(distances,**settings)[-1]["approach_decision"]["primary_object"]

    def test_positive_closing_and_fast_close(self):
        for distances in ([1,.9,.8,.7],[.9,.8,.68,.56],[.75,.65,.55,.45]):
            p=self.primary(distances)
            self.assertGreater(p["approach_speed_mps"],.5)
            self.assertEqual(p["state"],"FAST_APPROACH")
            self.assertTrue(p["avoid_candidate"])
            self.assertAlmostEqual(p["ttc_s"],p["distance_m"]/p["approach_speed_mps"])

    def test_receding_static_side_and_slow(self):
        p=self.primary([.4,.5,.6,.7])
        self.assertEqual(p["state"],"RECEDING");self.assertFalse(p["avoid_candidate"]);self.assertIsNone(p["ttc_s"])
        p=self.primary([.6,.61,.59,.6]);self.assertEqual(p["state"],"IN_RANGE_STATIC")
        self.assertFalse(p["avoid_candidate"])
        p=self.primary([.9,.88,.86,.84]);self.assertEqual(p["state"],"SLOW_APPROACH")
        self.assertFalse(p["avoid_candidate"])

    def test_one_frame_spike_and_small_history_unknown(self):
        for distances,settings in [([.5],{}),([.70,.71,.50],{}),([.9,.8,.7],{"fast_history_samples":2,"fast_required_decrease_frames":1})]:
            result=run(distances,**settings)[-1]["approach_decision"]
            self.assertIsNone(result["primary_object"]);self.assertFalse(result["avoid_candidate"])
            self.assertFalse(result["current_frame_valid"])

    def test_far_fast_and_observe_out_of_range(self):
        p=self.primary([1.2,1.1,1]);self.assertEqual(p["state"],"FAST_APPROACH");self.assertFalse(p["avoid_candidate"])
        result=run([1.4,1.3,1.21])[-1]["approach_decision"]
        self.assertEqual(result["objects"][0]["state"],"OUT_OF_RANGE")
        self.assertIsNone(result["primary_object"])

    def test_doppler_disagreement_not_veto(self):
        p=self.primary([.9,.8,.7],doppler=(.81,))
        self.assertTrue(p["avoid_candidate"]);self.assertIn("DOPPLER_DISAGREEMENT",p["evidence"])
        p=self.primary([.9,.8,.7],doppler=(-.81,))
        self.assertIn("DOPPLER_SUPPORT",p["evidence"])

    def test_miss_gap_new_id_and_paused_clear(self):
        result=run([.9,.8,None,.7])[-1]["approach_decision"]
        self.assertFalse(result["avoid_candidate"])
        p=RadarProcessor(settings_for("BALANCED"))
        p.process_frame(1,cloud(.9),0);p.process_frame(2,cloud(.8),.1)
        self.assertIsNone(p.process_frame(4,cloud(.7),.2)["approach_decision"]["primary_object"])
        p.reset();self.assertIsNone(p.process_frame(5,cloud(.7),.3)["approach_decision"]["primary_object"])
        p.set_moving(True,.4)
        result=p.process_frame(6,cloud(.6),.4)["approach_decision"]
        self.assertFalse(result["avoid_candidate"]);self.assertEqual(result["objects"],[])

    def test_off_observe_core_legacy_and_fast_unchanged(self):
        for fast_mode in ("OFF","OBSERVE","ENABLED"):
            off=run([.9,.8,.7,.6],approach_path_mode="OFF",fast_path_mode=fast_mode,doppler=(-.81,))
            observed=run([.9,.8,.7,.6],approach_path_mode="OBSERVE",fast_path_mode=fast_mode,doppler=(-.81,))
            for a,b in zip(off,observed):
                self.assertEqual(a["fast_path"],b["fast_path"])
                for name in ("target","legacy_target"):
                    for key in ("id","risk","distance","ttc","confidence","range_rate_velocity"):
                        self.assertEqual(a[name].get(key) if a[name] else None,b[name].get(key) if b[name] else None)
            self.assertIsNone(off[-1]["approach_decision"]["primary_object"])

    def test_boundaries_and_priority_independent_of_fast_settings(self):
        p=RadarProcessor(settings_for("BALANCED"))
        base=dict(id=1,point_count=1,raw_distance=.8,robust_range_rate=.5,consecutive_hits=3,raw_distance_history_count=3,
                  distance_delta_total=-.2,consecutive_distance_decreases=2,approach_doppler_median=0)
        close=p.object_approach_decision(base)
        self.assertTrue(close["avoid_candidate"])
        far=p.object_approach_decision(dict(base,id=2,raw_distance=1.2))
        self.assertFalse(far["avoid_candidate"])
        self.assertEqual(p.approach_output([far,close])["primary_object"]["track_id"],1)
        static=p.object_approach_decision(dict(base,robust_range_rate=.1))
        self.assertEqual(static["state"],"IN_RANGE_STATIC")
        result=run([.9,.8,.7],fast_min_range_rate_mps=100)[-1]
        self.assertFalse(result["fast_path"]["candidate"])
        self.assertTrue(result["approach_decision"]["avoid_candidate"])

    def test_settings_validation_and_effective_range(self):
        for settings in ({"approach_path_mode":"ENABLED"},{"approach_min_track_frames":3.0},
                         {"approach_avoid_max_distance_m":1.3},{"approach_speed_deadband_mps":.5},
                         {"fast_approach_speed_mps":float("nan")}):
            with self.assertRaises(ValueError):replace(settings_for("BALANCED"),**settings).validate()
        p=RadarProcessor(replace(settings_for("BALANCED"),max_range_m=.9))
        self.assertEqual(p.approach_output([])["effective_observe_distance_m"],.9)


if __name__ == "__main__":unittest.main()
