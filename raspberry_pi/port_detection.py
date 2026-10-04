from dataclasses import dataclass
from pathlib import Path
from serial.tools import list_ports


VID = 0x10C4
PID = 0xEA70


@dataclass(frozen=True)
class RadarPorts:
    cli: str
    data: str
    source: str


def detect_ports(no_auto=False, cli_override=None, data_override=None):
    if cli_override and data_override:
        return RadarPorts(cli_override, data_override, "manual")
    if no_auto:
        return RadarPorts(cli_override or "/dev/ttyUSB0", data_override or "/dev/ttyUSB1", "fallback")
    stable_cli, stable_data = Path("/dev/radar_cli"), Path("/dev/radar_data")
    if stable_cli.exists() and stable_data.exists():
        return RadarPorts(str(stable_cli), str(stable_data), "udev")
    enhanced = standard = None
    for p in list_ports.comports():
        if p.vid != VID or p.pid != PID:
            continue
        text = " ".join(str(x or "") for x in (p.description, p.interface, p.product)).lower()
        if "enhanced" in text:
            enhanced = p.device
        elif "standard" in text:
            standard = p.device
    if enhanced and standard:
        return RadarPorts(enhanced, standard, "cp2105")
    return RadarPorts(cli_override or "/dev/ttyUSB0", data_override or "/dev/ttyUSB1", "fallback")
