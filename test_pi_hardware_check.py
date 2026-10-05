import json
import struct
import uuid
import contextlib
import io
from pathlib import Path
import unittest
from unittest.mock import patch, MagicMock

from tools.pi_radar_hardware_check import Capture, classify, describe, sign_check, Deadline, main, fast_check, SCENARIOS
from tools.pi_radar_udp_probe import decode_packet, safe_view
from raspberry_pi.serial_source import RadarSerialSource, MAGIC_WORD
from raspberry_pi.profiles import settings_for
from radar_processing import RadarProcessor


@contextlib.contextmanager
def report_directory():
    # Default Windows Temp ACLs can prevent child access in restricted runners.
    directory = Path(__file__).parent / "logs" / ("software_test_" + uuid.uuid4().hex)
    directory.mkdir(parents=True)
    try:
        yield str(directory)
    finally:
        for run in directory.iterdir():
            for file in run.iterdir():
                file.unlink()
            run.rmdir()
        directory.rmdir()


class HardwareCheckTests(unittest.TestCase):
    def test_fast_verdict_requires_complete_scenarios_and_capture(self):
        scenes=[dict(name=name, capture_complete=True, action_window=dict(frames=20,fast_candidate_fraction=.1 if name=="SCENE_3_FAST_APPROACH" else 0,
                    legacy_target_fraction=0), false_positive_diagnostics=[], scenario_assessment=dict(status="PASS")) for name,_ in SCENARIOS]
        self.assertEqual(fast_check(scenes)["status"],"PASS")
        self.assertEqual(fast_check(scenes[:-1])["status"],"UNKNOWN")
        self.assertEqual(fast_check(scenes,"KeyboardInterrupt")["status"],"UNKNOWN")
        scenes[0]["capture_complete"]=False
        self.assertEqual(fast_check(scenes)["status"],"UNKNOWN")
        scenes[0]["capture_complete"]=True
        scenes[0]["action_window"]["frames"]=0
        self.assertEqual(fast_check(scenes)["status"],"UNKNOWN")

    def scene(self, name, distances, doppler):
        capture = Capture(name)
        processor = RadarProcessor(settings_for("BALANCED"))
        for frame, distance in enumerate(distances, 1):
            parsed = dict(frame=frame, points=[dict(x=0, y=distance, z=0, doppler=doppler, snr=12)])
            result = processor.process_frame(frame, parsed["points"], frame*.1)
            capture.add(parsed, frame*.1, result, .2, .3)
        result = capture.summary(len(distances)*.1)
        result["moving_doppler_median"] = doppler
        return result

    def test_raw_is_not_roi_filtered(self):
        result = self.scene("RAW", [2, 3, 4], -.8)
        self.assertEqual(result["raw_statistics"]["distance"]["count"], 3)
        self.assertEqual(result["stages"]["roi"]["total"], 0)
        self.assertEqual(result["largest_rejection_stage"], "reject_range")

    def test_sign_agreement_mismatch_and_unknown(self):
        fast = self.scene("SCENE_3_FAST_APPROACH", [1.2, .8, .4], -.8)
        recede = self.scene("SCENE_4_RECEDE", [.3, .6, .9], .8)
        def evidence(first, last, doppler):
            return dict(lifetime_frames=3, angle_span=0, raw_doppler_median=doppler, total_distance_change=last-first)
        fast["full_capture"]["tracks"] = [evidence(1.2,.4,-.8)]
        recede["full_capture"]["tracks"] = [evidence(.3,.9,.8)]
        self.assertEqual(sign_check([fast, recede], -1)["status"], "PASS")
        self.assertTrue(sign_check([fast, recede], 1)["possible_sign_mismatch"])
        self.assertEqual(sign_check([fast], -1)["status"], "UNKNOWN")
        fast["full_capture"]["tracks"] = []
        self.assertEqual(sign_check([fast, recede], -1)["status"], "UNKNOWN")

    def test_target_zero_does_not_fail_transport(self):
        result = self.scene("SCENE_1_STATIC_HAND", [.6]*3, 0)
        report = classify({}, [result], 10, .2, True, True, -1)
        self.assertEqual(report["checks"]["DATA STREAM"], "PASS")
        self.assertEqual(report["checks"]["POST PROCESSING"], "WARN")
        self.assertNotEqual(report["sensor_status"], "HARDWARE_PROBLEM")

    def test_continuity_wrap_gaps_duplicates_order(self):
        capture = Capture("sequence")
        capture.frames = [0xfffffffe, 0xffffffff, 0, 2, 2, 1]
        result = capture.summary(1)
        self.assertEqual((result["frame_gaps"], result["duplicates"], result["out_of_order"]), (1, 1, 1))
        self.assertEqual(describe([1, float("nan"), 3])["median"], 2)

    def test_stream_instrumentation_reuses_parser(self):
        good = struct.pack("<8s8I", MAGIC_WORD, 1, 40, 0, 1, 0, 0, 0, 0)
        invalid = bytearray(good)
        invalid[12:16] = (1).to_bytes(4, "little")
        source = RadarSerialSource("a", "b", "unused")
        source.data = MagicMock(in_waiting=0)
        source.data.read.side_effect = [b"noise" + invalid + good]
        stop = MagicMock()
        stop.is_set.side_effect = [False, True]
        frames = list(source.frames(stop))
        self.assertEqual(len(frames), 1)
        self.assertEqual(source.stream_stats["invalid_lengths"], 1)
        self.assertEqual(source.stream_stats["parsed_frames"], 1)
        self.assertGreater(source.stream_stats["resyncs"], 0)

    def test_parse_failures_are_reported(self):
        source = RadarSerialSource("a", "b", "unused")
        source.data = MagicMock(in_waiting=0)
        source.data.read.return_value = struct.pack("<8s8I", MAGIC_WORD, 1, 40, 0, 1, 0, 0, 0, 0)
        stop = MagicMock()
        stop.is_set.side_effect = [False, True]
        with patch("raspberry_pi.serial_source.parse_frame", return_value=None):
            self.assertEqual(list(source.frames(stop)), [])
        self.assertEqual(source.stream_stats["parse_failures"], 1)

    def test_cli_error_and_timeout_command_evidence(self):
        source = RadarSerialSource("a", "b", "unused")
        source.cli = MagicMock(in_waiting=1)
        source.cli.read.return_value = b"Error bad command"
        with self.assertRaises(RuntimeError):
            source._send_command("bad")
        self.assertEqual(source.command_results[-1]["status"], "ERROR")
        with patch("raspberry_pi.serial_source.time.monotonic", side_effect=[0, 2]):
            with self.assertRaises(TimeoutError):
                source._send_command("timeout")
        self.assertEqual(source.command_results[-1]["status"], "TIMEOUT")

    def test_udp_malformed_stale_invalid_safe(self):
        packet = dict(protocol_version=1, timestamp=1, frame=1, radar_health="OK", radar_valid=True,
                      target=dict(distance=.5, angle=0, range_rate_velocity=.8, ttc=.6, risk="AVOID"))
        decoded = decode_packet(json.dumps(packet).encode())
        self.assertIsNone(safe_view(decoded, 2, 0, 1)["target"])
        self.assertIsNone(safe_view(decoded, 0, 2, 1)["target"])
        decoded["radar_health"] = "CONFIG_ERROR"
        self.assertFalse(safe_view(decoded, 0, 0, 1)["radar_valid"])
        for data in (b"broken", b"[]", b'{"protocol_version":1,"timestamp":NaN}', b"null"):
            with self.assertRaises(ValueError):
                decode_packet(data)

    def test_deadline_ends_without_frames(self):
        with patch("tools.pi_radar_hardware_check.time.monotonic", side_effect=[1, 2, 4]):
            stop = Deadline(2)
            self.assertFalse(stop.is_set())
            self.assertTrue(stop.is_set())

    def test_report_files_with_fake_source_no_hardware(self):
        class FakeSource:
            def __init__(self, *args):
                self.command_results = [dict(command="sensorStart", status="DONE", response="Done")]
                self.stream_stats = dict(total_bytes=40, parsed_frames=1)
                self.parse_seconds = [.0001]
                self.configuration_complete = False
                self.data = MagicMock()
            def configure(self): self.configuration_complete = True
            def close(self): pass
            def frames(self, stop):
                yield dict(frame=1, points=[dict(x=0, y=.5, z=0, doppler=0, snr=12)]), 1.0
        with report_directory() as directory:
            with patch("tools.pi_radar_hardware_check.RadarSerialSource", FakeSource), contextlib.redirect_stdout(io.StringIO()):
                code = main(["--cli-port", "a", "--data-port", "b", "--duration", ".01", "--raw-log", "--diag-log", "--output-dir", directory])
            self.assertEqual(code, 0)
            run = next(Path(directory).iterdir())
            for name in ("summary.json", "test_report.txt", "raw_points.csv", "objects.csv", "frame_diagnostics.csv", "scenarios.csv"):
                self.assertTrue((run/name).is_file())
            summary = json.loads((run/"summary.json").read_text())
            self.assertEqual(summary["frame_count"], 1)
            self.assertEqual(summary["approach_sign_check"]["status"], "UNKNOWN")
            self.assertEqual(summary["raw_statistics"]["snr"]["median"], 12)

    def test_missing_data_returns_failure_not_sensor_ready(self):
        report = classify({}, [], 10, .2, True, True, -1)
        self.assertEqual(report["checks"]["DATA STREAM"], "FAIL")
        self.assertEqual(report["sensor_status"], "HARDWARE_PROBLEM")

    def test_truncated_tlv_is_not_silently_verified(self):
        source = RadarSerialSource("a", "b", "unused")
        source.data = MagicMock(in_waiting=0)
        source.data.read.return_value = struct.pack("<8s8I", MAGIC_WORD, 1, 40, 0, 1, 0, 0, 1, 0)
        stop = MagicMock()
        stop.is_set.side_effect = [False, True]
        self.assertEqual(len(list(source.frames(stop))), 1)
        self.assertEqual(source.stream_stats["truncated_tlv_envelopes"], 1)


if __name__ == "__main__":
    unittest.main()
