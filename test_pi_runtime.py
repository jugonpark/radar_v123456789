import json
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
        rt = RadarRuntime(c, source_factory=FakeSource, telemetry_factory=lambda *a, **k: TelemetrySender("h", 1, enabled=False))
        rt.connect(); self.assertEqual(rt.health, "OK")
        self.assertEqual(rt.processor.robot.state, "SETTLING")
        rt.last_frame_monotonic = 1.0
        self.assertEqual(rt.check_health(2.0), "STALE")
        rt.close()
        cfg.unlink(missing_ok=True)
        for item in log_dir.glob("*"): item.unlink()
        log_dir.rmdir()


if __name__ == "__main__": unittest.main()
