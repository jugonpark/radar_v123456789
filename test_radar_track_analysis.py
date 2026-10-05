import unittest
from tools.radar_track_analysis import TrackAnalysis, false_positive_diagnostics, scenario_assessment, approach_check, APPROACH_SCENE_NAMES, approach_console


class TrackAnalysisTests(unittest.TestCase):
    def test_approach_shared_metrics_signed_primary_population(self):
        a=TrackAnalysis(10)
        for i,speed in enumerate([None,.7,-.5]):
            primary=None if speed is None else dict(track_id=4,distance_m=.6,approach_speed_mps=speed,state="FAST_APPROACH" if speed>0 else "RECEDING",avoid_candidate=speed>0)
            a.add(i,10+i*.1,dict(objects=[],target=None,counts={},approach_decision=dict(decision="AVOID_CANDIDATE" if speed and speed>0 else "SAFE",current_frame_valid=primary is not None,primary_object=primary,objects=[primary] if primary else [],avoid_candidate=bool(speed and speed>0))))
        s=a.summary()
        self.assertAlmostEqual(s["avoid_candidate_frame_fraction"],1/3)
        self.assertAlmostEqual(s["approach_decision_frame_fraction"],1/3)
        self.assertAlmostEqual(s["fast_approach_frame_fraction"],1/3)
        self.assertAlmostEqual(s["median_approach_speed_mps"],.1)
        self.assertAlmostEqual(s["first_fast_approach_latency"],.1)
        self.assertEqual(s["primary_track_id"],4)
        self.assertIn("v=N/A",approach_console({}))

    def test_four_scene_approach_check_requires_complete_observations(self):
        scenes=[dict(name=n,capture_complete=True,action_window=dict(frames=20,avoid_candidate_frame_fraction=.1 if n=="SCENE_3_FAST_APPROACH" else 0,receding_frame_fraction=.1 if n=="SCENE_4_RECEDE" else 0),approach_false_positive_diagnostics=[]) for n in APPROACH_SCENE_NAMES]
        self.assertEqual(approach_check(scenes)["status"],"PASS")
        self.assertEqual(approach_check(scenes[:-1])["status"],"UNKNOWN")
        self.assertEqual(approach_check(scenes,"interrupted")["status"],"UNKNOWN")
        scenes[0]["approach_false_positive_diagnostics"]=[{"range_rate":.7}]
        self.assertEqual(approach_check(scenes)["status"],"WARN")

    def test_empty_capture_unknown_and_static_legacy_evidence_warns(self):
        a=TrackAnalysis()
        self.assertEqual(scenario_assessment("SCENE_0_EMPTY", a.summary())["status"],"UNKNOWN")
        obj=dict(id=1,raw_distance=.7,angle=0)
        a.add(1,0,dict(objects=[obj],target=obj,counts={}))
        self.assertEqual(scenario_assessment("SCENE_1_STATIC_HAND",a.summary())["status"],"WARN")

    def test_absent_frame_gap_and_temporal_gap_break_continuity(self):
        obj=dict(id=1,raw_distance=.7,angle=0,point_count=1)
        for sequence,expected in [([(1,0,True),(2,.1,False),(3,.2,True)],1),
                                  ([(1,0,True),(3,.1,True)],1),
                                  ([(1,0,True),(2,.1,True),(3,2,True)],2)]:
            a=TrackAnalysis()
            for frame,t,present in sequence:
                a.add(frame,t,dict(objects=[obj] if present else [],target=None,counts={}))
            self.assertEqual(a.summary()["max_continuous_lifetime"],expected)

    def test_retained_missed_track_is_not_observed(self):
        a=TrackAnalysis()
        a.add(1,0,dict(objects=[dict(id=1,angle=0,misses=1,point_count=0)],target=None,counts={}))
        self.assertEqual(a.summary()["tracks"],[])

    def test_fragmentation_and_comparison_and_false_positive(self):
        a = TrackAnalysis(1.0)
        for frame in range(4):
            obj = dict(id=frame, raw_distance=.7, angle=0, confidence="HIGH", fast_approach_candidate=True,
                       robust_range_rate=.8, raw_doppler_median=-.8, approach_doppler_median=.8)
            a.add(frame, 1+frame*.1, dict(objects=[obj], target=obj, legacy_target=None, counts=dict(track_new=1)))
        s = a.summary()
        self.assertEqual(s["legacy_target_fraction"], 0)
        self.assertEqual(s["legacy_high_fraction"], 1)
        self.assertEqual(s["fast_candidate_fraction"], 1)
        self.assertEqual(s["max_consecutive_fast_frames"], 4)
        self.assertEqual(s["track_fragmentation_estimate"], 1)
        self.assertIn("TRACK_MATCHING_MAY_BE_TOO_STRICT", s["diagnostics"])
        self.assertEqual(len(false_positive_diagnostics("SCENE_4_RECEDE", a)), 4)
        self.assertEqual(false_positive_diagnostics("SCENE_3_FAST_APPROACH", a), [])

    def test_representatives_and_latency(self):
        a = TrackAnalysis(4)
        for i, d in enumerate([1,.9,.8]):
            a.add(i, 4+i*.1, dict(objects=[dict(id=1, raw_distance=d, angle=i, robust_range_rate=1,
                fast_approach_candidate=i==2, consecutive_distance_decreases=i)], target=None, counts={}))
        s=a.summary()
        self.assertAlmostEqual(s["first_detection_latency_from_action_start"], .2)
        self.assertEqual(s["longest_track"]["lifetime_frames"], 3)
        self.assertAlmostEqual(s["largest_closing_track"]["total_distance_change"], -.2)


if __name__ == "__main__":
    unittest.main()
