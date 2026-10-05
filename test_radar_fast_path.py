import math
import unittest
from dataclasses import replace
from radar_processing import RadarProcessor, Settings
from raspberry_pi.profiles import settings_for


def cloud(d, doppler=(-.81, .81), angle=0):
    a=math.radians(angle)
    return [dict(x=d*math.sin(a), y=d*math.cos(a), z=0, doppler=v, snr=20) for v in doppler]


def run(distances, mode="OBSERVE", times=None, angles=None, doppler=(-.81,.81)):
    p=RadarProcessor(replace(settings_for("BALANCED"), fast_path_mode=mode))
    out=[]
    for i,d in enumerate(distances):
        out.append(p.process_frame(i+1, [] if d is None else cloud(d,doppler,angles[i] if angles else 0), times[i] if times else i*.1))
    return out


class FastTests(unittest.TestCase):
    def test_fast_mixed_doppler_and_all_pairwise_median(self):
        r=run([.9,.8,.7])[-1]; o=r["objects"][0]
        self.assertTrue(r["fast_path"]["candidate"])
        self.assertAlmostEqual(o["robust_range_rate"],1)
        self.assertFalse(r["fast_path"]["doppler_support"])
        self.assertIsNone(r["target"])
        self.assertEqual(o["consecutive_distance_decreases"],2)

    def test_off_observe_preserve_legacy(self):
        off=run([.85,.75,.65,.55],"OFF",doppler=(-.81,))
        observe=run([.85,.75,.65,.55],doppler=(-.81,))
        keys=("id","distance","ttc","risk","confidence")
        for a,b in zip(off,observe):
            self.assertEqual(None if a["target"] is None else [a["target"][k] for k in keys],None if b["target"] is None else [b["target"][k] for k in keys])
        self.assertFalse(off[-1]["fast_path"]["candidate"])

    def test_enabled_explicit_and_diagnostic_target_forbidden(self):
        r=run([.9,.8,.7],"ENABLED")[-1]
        self.assertIsNotNone(r["target"]); self.assertIsNone(r["legacy_target"])
        p=RadarProcessor(replace(settings_for("DIAGNOSTIC"),fast_path_mode="ENABLED"))
        for i,d in enumerate([.9,.8,.7]): r=p.process_frame(i+1,cloud(d),i*.1)
        self.assertTrue(r["fast_path"]["candidate"]); self.assertIsNone(r["target"])

    def test_static_slow_recede_side_single_frame(self):
        for ds in ([.7,.7,.7],[.7,.69,.68],[.5,.6,.7],[.7]):
            self.assertFalse(run(ds)[-1]["fast_path"]["candidate"])
        self.assertFalse(run([.7,.7,.7],angles=[0,5,10])[-1]["fast_path"]["candidate"])

    def test_doppler_disagreement_is_diagnostic_not_veto(self):
        r=run([.9,.8,.7],doppler=(.81,))[-1]
        self.assertTrue(r["fast_path"]["candidate"])
        self.assertIn("DOPPLER_DISAGREEMENT",r["fast_path"]["evidence"])
        self.assertIn("DOPPLER_DISAGREEMENT",r["fast_path"]["reason"])

    def test_five_sample_outlier_pairwise_median(self):
        # One raised sample remains inside unchanged association gates;
        # six clean pairwise slopes outweigh its four affected slopes.
        o=run([.9,.8,.75,.6,.5])[-1]["objects"][0]
        self.assertEqual(o["raw_distance_history_count"],5)
        self.assertAlmostEqual(o["robust_range_rate"],1.0)
        self.assertTrue(o["fast_approach_candidate"])

    def test_jitter_spike_no_trigger(self):
        self.assertFalse(run([.8,.79,.8,.65])[-1]["fast_path"]["candidate"])

    def test_miss_gap_nonmonotonic_and_frame_skip_break_evidence(self):
        self.assertFalse(run([.9,.8,None,.7])[-1]["fast_path"]["candidate"])
        self.assertFalse(run([.9,.8,.7],times=[0,.1,.7])[-1]["fast_path"]["candidate"])
        self.assertFalse(run([.9,.8,.7],times=[0,.1,.05])[-1]["fast_path"]["candidate"])
        p=RadarProcessor(settings_for("BALANCED"))
        p.process_frame(1,cloud(.9),0); p.process_frame(2,cloud(.8),.1)
        self.assertFalse(p.process_frame(4,cloud(.7),.2)["fast_path"]["candidate"])

    def test_angle_fragmentation_and_bounded_history(self):
        self.assertFalse(run([.9,.8,.7],angles=[0,10,20])[-1]["fast_path"]["candidate"])
        self.assertFalse(run([.9,.8,.7],angles=[0,30,0])[-1]["fast_path"]["candidate"])
        o=run([.9-i*.03 for i in range(15)])[-1]["objects"][0]
        self.assertEqual(o["raw_distance_history_count"],5)
        self.assertLessEqual(o["distance_history_count"],8)

    def test_reset_and_single_point_persistence(self):
        r=run([.9,.8,.7],doppler=(.81,))[-1]
        self.assertTrue(r["fast_path"]["candidate"])
        p=RadarProcessor(settings_for("BALANCED"))
        p.process_frame(1,cloud(.9),0); p.process_frame(2,cloud(.8),.1);p.reset()
        self.assertFalse(p.process_frame(3,cloud(.7),.2)["fast_path"]["candidate"])

    def test_aliases_and_configuration_validation(self):
        r=run([.7])[-1]
        self.assertEqual(r["raw"][0]["raw_doppler_mps"],-.81)
        self.assertEqual(r["raw"][0]["approach_doppler_mps"],.81)
        self.assertEqual(Settings().fast_path_mode,"OFF")
        for kwargs in ({"fast_path_mode":"AUTO"},{"fast_history_samples":1},{"fast_min_track_frames":1},{"fast_min_range_rate_mps":float("nan")}):
            with self.assertRaises(ValueError): replace(Settings(),**kwargs).validate()

if __name__=="__main__": unittest.main()
