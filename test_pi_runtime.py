import json
import csv
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from raspberry_pi.pi_config import PiConfig
from raspberry_pi.port_detection import RadarPorts, detect_ports
from raspberry_pi.telemetry_sender import TelemetrySender
from raspberry_pi.runtime import RadarRuntime


class FakeSocket:
    def __init__(self): self.sent = []
    def sendto(self, data, address): self.sent.append((json.loads(data), address))
    def close(self): pass


class FakeSource:
    def __init__(self, *args): self.closed = False
    def configure(self): pass
    def close(self): self.closed = True


class PiTests(unittest.TestCase):
    def test_approach_independent_overrides_csv_and_telemetry(self):
        root=Path(__file__).parent
        log_dir=root/"logs"/"approach_runtime_test"
        sock=FakeSocket()
        runtime=RadarRuntime(PiConfig(log_dir=log_dir), source_factory=FakeSource,
            telemetry_factory=lambda *args:TelemetrySender("h",1,sock=sock),
            approach_settings={"fast_approach_speed_mps":.4}, fast_settings={"fast_min_range_rate_mps":100})
        try:
            runtime.health="OK"
            for i,d in enumerate([.9,.8,.7]):
                result=runtime.process(dict(frame=i+1,points=[dict(x=0,y=d,z=0,doppler=.81,snr=20)]),i*.1)
            self.assertTrue(result["approach_decision"]["avoid_candidate"])
            self.assertFalse(result["fast_path"]["candidate"])
            self.assertTrue(sock.sent[-1][0]["approach_decision"]["avoid_candidate"])
            with next(log_dir.glob("radar_objects_*.csv")).open(encoding="utf-8",newline="") as f:
                rows=list(csv.DictReader(f))
            self.assertEqual(rows[-1]["approach_state"],"FAST_APPROACH")
            self.assertGreater(float(rows[-1]["approach_speed_mps"]),.4)
            runtime.health="STALE"
            cleared=runtime.process(dict(frame=4,points=[dict(x=0,y=.6,z=0,doppler=.81,snr=20)]),.3)
            self.assertIsNone(cleared["approach_decision"]["primary_object"])
            self.assertFalse(cleared["approach_decision"]["avoid_candidate"])
        finally:
            runtime.close()
            for file in log_dir.iterdir():file.unlink()
            log_dir.rmdir()

    def test_approach_cli_overrides(self):
        from raspberry_pi.pi_radar_main import build_parser
        args=build_parser().parse_args(["--approach-path-mode","OBSERVE","--approach-fast-speed",".7","--approach-avoid-distance",".6"])
        self.assertEqual(args.fast_approach_speed_mps,.7)
        self.assertEqual(args.approach_avoid_max_distance_m,.6)

    def test_manual_and_fallback_ports(self):
        self.assertEqual(detect_ports(True).cli, "/dev/ttyUSB0")
        self.assertEqual(detect_ports(True, "/x", "/y"), RadarPorts("/x", "/y", "manual"))

    def test_cp2105_detection(self):
        p1 = MagicMock(device="/a", vid=0x10C4, pid=0xEA70, description="CP2105", interface="Enhanced")
        p2 = MagicMock(device="/b", vid=0x10C4, pid=0xEA70, description="CP2105", interface="Standard")
        with patch("raspberry_pi.port_detection.list_ports.comports", return_value=[p1, p2]), patch("raspberry_pi.port_detection.Path.exists", return_value=False):
            self.assertEqual(detect_ports().cli, "/a")
            self.assertEqual(detect_ports().data, "/b")

    def test_telemetry_rate_and_schema(self):
        sock = FakeSocket(); sender = TelemetrySender("h", 1, hz=10, sock=sock)
        payload = {"protocol_version": 1, "target": None}
        self.assertTrue(sender.send(payload, now=1.0)); self.assertFalse(sender.send(payload, now=1.01)); self.assertTrue(sender.send(payload, now=1.11))
        self.assertEqual(sock.sent[0][0]["protocol_version"], 1)

    def test_runtime_connect_and_stale_fail_safe(self):
        root = Path(__file__).parent
        cfg = root / ".pi_test_cfg"; cfg.write_text("sensorStop\n")
        log_dir = root / ".pi_test_logs"
        c = PiConfig(cfg_path=cfg, log_dir=log_dir, telemetry_hz=10)
        rt = RadarRuntime(c, source_factory=FakeSource,
                          telemetry_factory=lambda *a, **k: TelemetrySender("h", 1, enabled=False),
                          diagnostic_logging=True)
        rt.connect(); self.assertEqual(rt.health, "OK")
        self.assertEqual(rt.processor.robot.state, "SETTLING")
        observed = rt.process({"frame": 1, "points": [dict(x=0, y=.6, z=0, doppler=-.81, snr=12)]},
                              rt.processor.robot.transition_time + .4)
        self.assertEqual(observed["counts"]["tentative"], 1)
        self.assertTrue(list(log_dir.glob("radar_frame_diagnostics_*.csv")))
        rt.last_frame_monotonic = 1.0
        self.assertEqual(rt.check_health(2.0), "STALE")
        rt.close()
        cfg.unlink(missing_ok=True)
        for item in log_dir.glob("*"): item.unlink()
        log_dir.rmdir()


if __name__ == "__main__": unittest.main()
