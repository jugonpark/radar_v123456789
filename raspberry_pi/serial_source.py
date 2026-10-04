import time
from collections import deque
from pathlib import Path

import serial

from radar_gui_v1_ready import MAGIC_WORD, HEADER_SIZE, parse_frame


class RadarSerialSource:
    def __init__(self, cli_port, data_port, cfg_path, cli_baud=115200, data_baud=921600):
        self.cli_port, self.data_port, self.cfg_path = cli_port, data_port, Path(cfg_path)
        self.cli_baud, self.data_baud = cli_baud, data_baud
        self.cli = self.data = None

    def configure(self):
        if not self.cfg_path.is_file():
            raise FileNotFoundError(self.cfg_path)
        self.cli = serial.Serial(self.cli_port, self.cli_baud, timeout=0.2, write_timeout=1)
        self._send_command("sensorStop")
        for line in self.cfg_path.read_text(encoding="utf-8", errors="ignore").splitlines():
            line = line.strip()
            if line and not line.startswith("%"):
                self._send_command(line)
        self.data = serial.Serial(self.data_port, self.data_baud, timeout=0.05)

    def _send_command(self, command):
        self.cli.reset_input_buffer()
        self.cli.write((command + "\n").encode())
        deadline = time.monotonic() + 1.5
        response = bytearray()
        while time.monotonic() < deadline:
            response.extend(self.cli.read(self.cli.in_waiting or 1))
            text = response.decode(errors="ignore")
            if "Error" in text or "error" in text:
                raise RuntimeError(f"CLI Error: {command}: {text[-300:]}")
            if "Done" in text or "mmwDemo:/>" in text:
                return text
        raise TimeoutError(f"CLI response timeout: {command}: {response.decode(errors='ignore')[-200:]}")

    def frames(self, stop_event):
        buffer = bytearray()
        while not stop_event.is_set():
            buffer.extend(self.data.read(self.data.in_waiting or 1))
            while True:
                index = buffer.find(MAGIC_WORD)
                if index < 0:
                    if len(buffer) > 8192:
                        del buffer[:-7]
                    break
                if index:
                    del buffer[:index]
                if len(buffer) < HEADER_SIZE:
                    break
                total = int.from_bytes(buffer[12:16], "little")
                if total < HEADER_SIZE or total > 65536:
                    del buffer[0]
                    continue
                if len(buffer) < total:
                    break
                packet = bytes(buffer[:total])
                del buffer[:total]
                parsed = parse_frame(packet)
                if parsed is not None:
                    yield parsed, time.monotonic()

    def close(self):
        for port in (self.data, self.cli):
            if port:
                try:
                    port.close()
                except Exception:
                    pass
        self.data = self.cli = None
