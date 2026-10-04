from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


@dataclass
class PiConfig:
    cli_port: str = "/dev/ttyUSB0"
    data_port: str = "/dev/ttyUSB1"
    cli_baud: int = 115200
    data_baud: int = 921600
    cfg_path: Path = ROOT / "profile_3d_aop.cfg"
    log_dir: Path = ROOT / "logs"
    telemetry_host: str = "127.0.0.1"
    telemetry_port: int = 8890
    telemetry_hz: float = 10.0
    data_timeout_s: float = 0.5
    reconnect_interval_s: float = 2.0
    object_log_max_bytes: int = 10 * 1024 * 1024
    raw_logging: bool = False
