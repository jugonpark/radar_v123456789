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
        self.command_results = []
        self.configuration_complete = False
        self.stream_stats = dict(total_bytes=0, valid_packets=0, parsed_frames=0,
                                 parse_failures=0, invalid_lengths=0, resyncs=0,
                                 point_count_mismatches=0, truncated_tlv_envelopes=0)
        self.parse_seconds = deque(maxlen=10000)

    def configure(self):
        if not self.cfg_path.is_file():
            raise FileNotFoundError(self.cfg_path)
        self.cli = serial.Serial(self.cli_port, self.cli_baud, timeout=0.2, write_timeout=1)
        self._send_command("sensorStop")
        for line in self.cfg_path.read_text(encoding="utf-8", errors="ignore").splitlines():
            line = line.strip()
            if line and not line.startswith("%"):
                self._send_command(line)
        self.configuration_complete = True
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
                self.command_results.append(dict(command=command, status="ERROR", response=text))
                raise RuntimeError(f"CLI Error: {command}: {text[-300:]}")
            if "Done" in text or "mmwDemo:/>" in text:
                self.command_results.append(dict(command=command, status="DONE" if "Done" in text else "PROMPT", response=text))
                return text
        self.command_results.append(dict(command=command, status="TIMEOUT", response=response.decode(errors="ignore")))
        raise TimeoutError(f"CLI response timeout: {command}: {response.decode(errors='ignore')[-200:]}")

    def frames(self, stop_event):
        buffer = bytearray()
        while not stop_event.is_set():
            chunk = self.data.read(self.data.in_waiting or 1)
            self.stream_stats["total_bytes"] += len(chunk)
            buffer.extend(chunk)
            while True:
                index = buffer.find(MAGIC_WORD)
                if index < 0:
                    if len(buffer) > 8192:
                        self.stream_stats["resyncs"] += 1
                        del buffer[:-7]
                    break
                if index:
                    self.stream_stats["resyncs"] += 1
                    del buffer[:index]
                if len(buffer) < HEADER_SIZE:
                    break
                total = int.from_bytes(buffer[12:16], "little")
                if total < HEADER_SIZE or total > 65536:
                    self.stream_stats["invalid_lengths"] += 1
                    del buffer[0]
                    continue
                if len(buffer) < total:
                    break
                packet = bytes(buffer[:total])
                del buffer[:total]
                self.stream_stats["valid_packets"] += 1
                # Audit bounds using the working parser's payload-length convention.
                # This does not decode or change any TLV payload.
                offset = HEADER_SIZE
                for _ in range(int.from_bytes(packet[32:36], "little")):
                    if offset + 8 > total:
                        self.stream_stats["truncated_tlv_envelopes"] += 1
                        break
                    length = int.from_bytes(packet[offset+4:offset+8], "little")
                    offset += 8 + length
                    if offset > total:
                        self.stream_stats["truncated_tlv_envelopes"] += 1
                        break
                started = time.monotonic()
                parsed = parse_frame(packet)
                self.parse_seconds.append(time.monotonic() - started)
                if parsed is not None:
                    self.stream_stats["parsed_frames"] += 1
                    if len(parsed["points"]) != parsed.get("num_obj", len(parsed["points"])):
                        self.stream_stats["point_count_mismatches"] += 1
                    yield parsed, time.monotonic()
                else:
                    self.stream_stats["parse_failures"] += 1

    def close(self):
        for port in (self.data, self.cli):
            if port:
                try:
                    port.close()
                except Exception:
                    pass
        self.data = self.cli = None
