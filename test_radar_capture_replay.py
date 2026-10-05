import csv
import unittest
from pathlib import Path
from unittest.mock import patch
from test_pi_hardware_check import report_directory
from tools.replay_radar_capture import load_capture, replay


class ReplayTests(unittest.TestCase):
    def test_replay_new_approach_settings_independent_of_v2(self):
        from dataclasses import replace
        from raspberry_pi.profiles import settings_for
        rows=[dict(scenario="S",frame=i+1,timestamp=i*.1,points=[dict(x=0,y=d,z=0,doppler=.81,snr=20)]) for i,d in enumerate([.9,.8,.7,.6])]
        settings=replace(settings_for("BALANCED"),fast_min_range_rate_mps=100,fast_approach_speed_mps=.5)
        scenes,diagnostics=replay(rows,{},settings)
        summary=scenes[0]["full_capture"]
        self.assertEqual(summary["fast_candidate_fraction"],0)
        self.assertGreater(summary["avoid_candidate_frame_fraction"],0)
        self.assertEqual(diagnostics[-1]["approach_state"],"FAST_APPROACH")
        suppressed,_=replay(rows,{},replace(settings,approach_avoid_max_distance_m=.5))
        self.assertEqual(suppressed[0]["full_capture"]["avoid_candidate_frame_fraction"],0)

    def test_duplicate_empty_receive_events_are_preserved(self):
        with report_directory() as directory:
            run=Path(directory)/"run";run.mkdir()
            self.write(run/"raw_points.csv", ["scenario","frame","received_monotonic","x","y","z"], [["X",1,1.1,0,.7,0]])
            self.write(run/"frame_diagnostics.csv", ["scenario","frame","received_monotonic"], [["X",1,1],["X",1,1.1],["X",2,1.2]])
            rows,_,_=load_capture(run)
            self.assertEqual([r["timestamp"] for r in rows],[1,1.1,1.2])
            self.assertEqual([len(r["points"]) for r in rows],[0,1,0])

    def test_ambiguous_nonfinite_and_mixed_timing_rejected(self):
        with report_directory() as directory:
            run=Path(directory)/"run";run.mkdir()
            self.write(run/"raw_points.csv", ["scenario","frame","received_monotonic"], [])
            for rows,message in [([["X",1,1],["X",1,1]],"Ambiguous"),
                                 ([["X",1,"nan"]],"Nonfinite"),
                                 ([["X",1,1],["X",2,""]],"Mixed")]:
                self.write(run/"frame_diagnostics.csv", ["scenario","frame","received_monotonic"],rows)
                with self.assertRaisesRegex(ValueError,message):
                    load_capture(run,expected_frame_period=.1)

    def write(self, path, header, rows):
        with path.open("w", newline="", encoding="utf-8") as f:
            writer=csv.writer(f); writer.writerow(header); writer.writerows(rows)

    def test_authoritative_empty_frames_and_real_timing(self):
        with report_directory() as directory:
            run=Path(directory)/"run";run.mkdir()
            self.write(run/"raw_points.csv", ["scenario","frame","received_monotonic","x","y","z","doppler"], [["S",1,10,0,.7,0,-.8]])
            self.write(run/"frame_diagnostics.csv", ["scenario","frame","received_monotonic"], [["S",1,10],["S",2,10.13]])
            rows,meta,warnings=load_capture(run)
            self.assertEqual(len(rows),2);self.assertEqual(rows[1]["points"],[])
            self.assertAlmostEqual(rows[1]["timestamp"]-rows[0]["timestamp"],.13)
            self.assertEqual(warnings,[])

    def test_missing_timing_requires_explicit_period(self):
        with report_directory() as directory:
            run=Path(directory)/"run";run.mkdir()
            self.write(run/"raw_points.csv", ["scenario","frame","x","y","z"], [["S",1,0,.7,0]])
            with self.assertRaisesRegex(ValueError,"expected-frame-period"):
                load_capture(run)
            rows,_,warnings=load_capture(run,expected_frame_period=.2)
            self.assertEqual(rows[0]["timestamp"],0)
            self.assertEqual(len(warnings),2)

    def test_action_window_separate_and_baseline_preserved(self):
        class Processor:
            def __init__(self,settings):pass
            def process_frame(self,frame,points,timestamp):
                obj=dict(id=1,raw_distance=.5,angle=0,fast_approach_candidate=True,confidence="LOW")
                return dict(objects=[obj],target=obj,legacy_target=None,counts={})
        with patch("tools.replay_radar_capture.RadarProcessor",Processor):
            scenes,_=replay([dict(scenario="S",frame=i,timestamp=t,points=[]) for i,t in enumerate([10,10.5,13])], {"S":dict(action_start_monotonic="10")},None,2)
        self.assertEqual(scenes[0]["full_capture"]["longest_track"]["lifetime_frames"],3)
        self.assertEqual(scenes[0]["action_window"]["longest_track"]["lifetime_frames"],2)
        self.assertEqual(scenes[0]["full_capture"]["legacy_target_fraction"],0)
        self.assertEqual(scenes[0]["timing"]["source"],"RECORDED_ACTION_START")


if __name__ == "__main__":
    unittest.main()
