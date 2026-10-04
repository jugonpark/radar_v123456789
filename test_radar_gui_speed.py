import unittest

from radar_gui import approach_label


class ApproachLabelTests(unittest.TestCase):
    def test_fast_and_slow_confirmed_approach(self):
        target = {"confirmed": True, "confidence": "HIGH",
                  "direction_state": "APPROACHING", "range_rate_velocity": 0.34}
        self.assertEqual(approach_label(target, 0.35), "SLOW")
        target["range_rate_velocity"] = 0.35
        self.assertEqual(approach_label(target, 0.35), "FAST")

    def test_unknown_when_unconfirmed_lost_or_invalid(self):
        target = {"confirmed": True, "confidence": "HIGH",
                  "direction_state": "APPROACHING", "range_rate_velocity": 0.8}
        for change in ({"confirmed": False}, {"direction_state": "LOST_TEMPORARY"},
                       {"range_rate_velocity": None}, {"range_rate_velocity": float("nan")}):
            self.assertEqual(approach_label(target | change, 0.35), "UNKNOWN")
        self.assertEqual(approach_label(None, 0.35), "UNKNOWN")


if __name__ == "__main__":
    unittest.main()
