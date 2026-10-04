import csv
import time
from datetime import datetime
from pathlib import Path

from radar_processing import RadarProcessor
try:
    from .pi_config import PiConfig
    from .port_detection import detect_ports
    from .robot_motion import RobotMotionProvider
    from .serial_source import RadarSerialSource
    from .telemetry_sender import TelemetrySender
except ImportError:
    from pi_config import PiConfig
    from port_detection import detect_ports
    from robot_motion import RobotMotionProvider
    from serial_source import RadarSerialSource
    from telemetry_sender import TelemetrySender


HEALTH_BAD = {"STALE", "DISCONNECTED", "CONFIG_ERROR", "ERROR"}


class RadarRuntime:
    def __init__(self, config=None, source_factory=RadarSerialSource, telemetry_factory=TelemetrySender,
                 motion_provider=None, no_auto_port=False, cli_override=None, data_override=None,
                 telemetry_enabled=True, raw_logging=None):
        self.config = config or PiConfig()
        self.source_factory, self.telemetry_factory = source_factory, telemetry_factory
        self.motion = motion_provider or RobotMotionProvider()
        ports = detect_ports(no_auto_port, cli_override, data_override)
        self.cli_port, self.data_port, self.port_source = ports.cli, ports.data, ports.source
        self.health = "CONNECTING"
        self.processor = RadarProcessor()
        self.source = None
        self.telemetry = telemetry_factory(self.config.telemetry_host, self.config.telemetry_port,
                                           self.config.telemetry_hz, telemetry_enabled)
        self.last_frame_monotonic = None
        self.last_reconnect = 0.0
        self.frame_count = 0
        self.start_monotonic = time.monotonic()
        self.object_file = None
        self.object_writer = None
        self.raw_file = None
        self.raw_writer = None
        self.raw_logging = self.config.raw_logging if raw_logging is None else raw_logging
        self._open_logs()

    def _open_logs(self):
        self.config.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.object_file = (self.config.log_dir / f"radar_objects_{stamp}.csv").open("w", newline="", encoding="utf-8")
        self.object_writer = csv.writer(self.object_file)
        self.object_writer.writerow(["timestamp", "frame", "robot_state", "object_id", "confirmed", "persistence", "confidence", "motion_state", "direction_state", "point_count", "centroid_x", "centroid_y", "centroid_z", "distance", "angle", "doppler_velocity", "range_rate_velocity", "ttc", "ttc_velocity_source", "risk", "target_selected"])
        if self.raw_logging:
            self.raw_file = (self.config.log_dir / f"radar_raw_{stamp}.csv").open("w", newline="", encoding="utf-8")
            self.raw_writer = csv.writer(self.raw_file)
            self.raw_writer.writerow(["timestamp", "frame", "point_id", "x", "y", "z", "doppler", "snr", "noise"])

    def connect(self):
        self.health = "CONFIGURING"
        self.source = self.source_factory(self.cli_port, self.data_port, self.config.cfg_path,
                                          self.config.cli_baud, self.config.data_baud)
        try:
            self.source.configure()
        except Exception:
            self.health = "CONFIG_ERROR"
            self.source.close()
            self.source = None
            raise
        self.last_frame_monotonic = None
        self.processor.reset()
        now = time.monotonic()
        self.processor.set_moving(True, now)
        self.processor.set_moving(False, now)
        self.health = "OK"

    def process(self, parsed, received_monotonic=None):
        received_monotonic = time.monotonic() if received_monotonic is None else received_monotonic
        self.last_frame_monotonic = received_monotonic
        self.frame_count += 1
        moving = self.motion.is_moving()
        self.processor.set_moving(moving, received_monotonic)
        result = self.processor.process_frame(parsed["frame"], parsed["points"], received_monotonic)
        if moving:
            result["target"] = None
        if self.health == "OK":
            self._log(result, parsed, received_monotonic)
            self._send_telemetry(result, received_monotonic)
        return result

    def check_health(self, now=None):
        now = time.monotonic() if now is None else now
        if self.health == "OK" and (self.last_frame_monotonic is None or now - self.last_frame_monotonic > self.config.data_timeout_s):
            self.health = "STALE"
            self.processor.reset()
        return self.health

    def _safe_result(self, result):
        if self.health in HEALTH_BAD:
            result["target"] = None
            result["risk"] = "N/A"
            result["radar_valid"] = False
        return result

    def _log(self, result, parsed, now):
        wall = time.time()
        for obj in result["objects"]:
            x, y, z = obj["centroid"]
            self.object_writer.writerow([wall, parsed["frame"], result["robot_state"], obj["id"], int(obj["confirmed"]), obj["persistence"], obj["confidence"], obj["motion_state"], obj["direction_state"], obj["point_count"], x, y, z, obj["distance"], obj["angle"], obj["doppler_velocity"], obj["range_rate_velocity"], obj["ttc"], obj["ttc_velocity_source"], obj["risk"], int(obj["target_selected"])])
        self.object_file.flush()
        if self.object_file.tell() >= self.config.object_log_max_bytes:
            self.object_file.close()
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.object_file = (self.config.log_dir / f"radar_objects_{stamp}.csv").open("w", newline="", encoding="utf-8")
            self.object_writer = csv.writer(self.object_file)
            self.object_writer.writerow(["timestamp", "frame", "robot_state", "object_id", "confirmed", "persistence", "confidence", "motion_state", "direction_state", "point_count", "centroid_x", "centroid_y", "centroid_z", "distance", "angle", "doppler_velocity", "range_rate_velocity", "ttc", "ttc_velocity_source", "risk", "target_selected"])
        if self.raw_writer:
            for i, p in enumerate(parsed["points"]):
                self.raw_writer.writerow([wall, parsed["frame"], i, p.get("x"), p.get("y"), p.get("z"), p.get("doppler"), p.get("snr"), p.get("noise")])
            self.raw_file.flush()

    def _send_telemetry(self, result, now):
        target = result.get("target")
        target_out = None if target is None else {k: target.get(k) for k in ("id", "distance", "angle", "doppler_velocity", "range_rate_velocity", "confidence", "ttc", "risk")}
        payload = {"protocol_version": 1, "timestamp": time.time(), "frame": result["frame_id"], "radar_health": self.health, "radar_valid": self.health == "OK", "robot_state": result["robot_state"], "processing_state": result["processing"], "counters": result["counts"], "target": target_out}
        self.telemetry.send(payload, now)

    def close(self):
        if self.source:
            self.source.close()
        self.telemetry.close()
        for handle in (self.object_file, self.raw_file):
            if handle:
                handle.close()
