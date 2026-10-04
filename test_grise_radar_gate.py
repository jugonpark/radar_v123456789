import unittest

from grise_radar_gate import GateSettings, VideoEvidence, decide


class GateTests(unittest.TestCase):
    def setUp(self):
        self.video = VideoEvidence(10.0, True, True, True, False, False)
        self.radar = {"timestamp": 10.0, "processing": "ACTIVE", "robot_state": "MONITORING",
                      "target": {"id": 7, "confirmed": True, "confidence": "HIGH",
                                 "direction_state": "APPROACHING", "distance": .65,
                                 "range_rate_velocity": .7, "ttc": .93}}

    def test_fast_approach_with_clear_video_sector(self):
        result = decide(self.radar, self.video, 10.05, True)
        self.assertEqual((result.action, result.direction), ("REQUEST_AVOID", "LEFT"))

    def test_slow_approach_monitors(self):
        self.radar["target"]["range_rate_velocity"] = .2
        self.assertEqual(decide(self.radar, self.video, 10.05, True).action, "MONITOR")

    def test_fails_closed_for_stale_missing_or_moving(self):
        self.assertEqual(decide(self.radar, self.video, 10.4, True).action, "STOP")
        self.assertEqual(decide(self.radar, None, 10.05, True).action, "STOP")
        self.assertEqual(decide(self.radar, self.video, 10.05, False).action, "STOP")
        self.radar["processing"] = "PAUSED"
        self.assertEqual(decide(self.radar, self.video, 10.05, True).action, "STOP")

    def test_no_clearance_stops(self):
        video = VideoEvidence(10, True, True, False, False, False)
        self.assertEqual(decide(self.radar, video, 10.05, True).action, "STOP")

    def test_missing_range_rate_stops(self):
        self.radar["target"]["range_rate_velocity"] = None
        self.assertEqual(decide(self.radar, self.video, 10.05, True).action, "STOP")


if __name__ == "__main__":
    unittest.main()
