import unittest
from raspberry_pi.pi_radar_main import build_parser
from raspberry_pi.profiles import settings_for
from raspberry_pi.runtime import OBJECT_COLUMNS

class PiFastTests(unittest.TestCase):
    def test_profile_modes_and_cli_explicit(self):
        self.assertEqual(settings_for("STRICT").fast_path_mode,"OFF")
        for name in ("BALANCED","DIAGNOSTIC"): self.assertEqual(settings_for(name).fast_path_mode,"OBSERVE")
        args=build_parser().parse_args(["--fast-path-mode","ENABLED","--fast-min-track-frames","4"])
        self.assertEqual(args.fast_path_mode,"ENABLED");self.assertEqual(args.fast_min_track_frames,4)
    def test_gui_apply_preserves_hidden_fast_settings(self):
        from types import SimpleNamespace
        from unittest.mock import patch, MagicMock
        from radar_gui import RadarApp
        from radar_processing import RadarProcessor
        settings=settings_for("BALANCED")
        var=MagicMock();var.get.return_value="0.15"
        threshold=MagicMock();threshold.get.return_value="0.5"
        fake=SimpleNamespace(processor=RadarProcessor(settings),settings_vars={"min_range_m":var},
                             fast_threshold_var=threshold,mode="LIVE",status_var=MagicMock(),fast_threshold_mps=.5)
        with patch("radar_gui.messagebox.showerror"):
            self.assertTrue(RadarApp._apply(fake))
        self.assertEqual(fake.processor.settings.fast_path_mode,"OBSERVE")

    def test_csv_is_scalar_summary(self):
        self.assertIn("robust_range_rate",OBJECT_COLUMNS)
        self.assertIn("raw_distance_history_count",OBJECT_COLUMNS)
        self.assertNotIn("raw_distance_history",OBJECT_COLUMNS)
if __name__=="__main__": unittest.main()
