import csv
import math
import os
import queue
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports


# ============================================================
# TI IWR6843AOP / mmWave demo packet constants
# ============================================================

MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"
HEADER_SIZE = 40

TLV_DETECTED_POINTS = 1
TLV_SIDE_INFO = 7


# ============================================================
# Adjustable risk / filter settings
# ============================================================

@dataclass
class RadarSettings:
    approach_sign: float = -1.0

    doppler_deadband_mps: float = 0.08

    min_range_m: float = 0.15
    max_range_m: float = 1.50

    min_snr_db: float = 10.0

    # Robot front FOV. abs(angle) <= fov_half_angle_deg
    fov_half_angle_deg: float = 75.0

    ttc_watch_s: float = 3.0
    ttc_warning_s: float = 2.0
    ttc_avoid_s: float = 1.2
    ttc_stop_s: float = 0.6

    emergency_distance_m: float = 0.20


# ============================================================
# Packet parsing
# ============================================================

def parse_frame(packet: bytes):
    if len(packet) < HEADER_SIZE:
        return None

    try:
        header = struct.unpack_from("<8s8I", packet, 0)
    except struct.error:
        return None

    if header[0] != MAGIC_WORD:
        return None

    frame_number = header[4]
    num_detected_obj = header[6]
    num_tlvs = header[7]

    offset = HEADER_SIZE
    points = []
    side_info = []

    for _ in range(num_tlvs):
        if offset + 8 > len(packet):
            break

        try:
            tlv_type, tlv_length = struct.unpack_from("<II", packet, offset)
        except struct.error:
            break

        offset += 8
        payload_start = offset

        # This keeps the same interpretation as the working receiver.
        payload_end = offset + tlv_length

        if payload_end > len(packet):
            break

        if tlv_type == TLV_DETECTED_POINTS:
            point_size = 16
            available_points = tlv_length // point_size
            count = min(num_detected_obj, available_points)

            for i in range(count):
                p_offset = payload_start + i * point_size
                if p_offset + point_size > payload_end:
                    break
                try:
                    x, y, z, doppler = struct.unpack_from(
                        "<ffff", packet, p_offset
                    )
                except struct.error:
                    break

                points.append({
                    "x": x,
                    "y": y,
                    "z": z,
                    "doppler": doppler,
                })

        elif tlv_type == TLV_SIDE_INFO:
            side_size = 4
            available_side = tlv_length // side_size
            count = min(num_detected_obj, available_side)

            for i in range(count):
                s_offset = payload_start + i * side_size
                if s_offset + side_size > payload_end:
                    break

                try:
                    snr_raw, noise_raw = struct.unpack_from(
                        "<HH", packet, s_offset
                    )
                except struct.error:
                    break

                side_info.append({
                    "snr": snr_raw / 10.0,
                    "noise": noise_raw / 10.0,
                })

        offset = payload_end

    for i, point in enumerate(points):
        if i < len(side_info):
            point["snr"] = side_info[i]["snr"]
            point["noise"] = side_info[i]["noise"]
        else:
            point["snr"] = None
            point["noise"] = None

    return {
        "frame": frame_number,
        "num_obj": num_detected_obj,
        "num_tlvs": num_tlvs,
        "points": points,
    }


# ============================================================
# Point processing
# ============================================================

def classify_risk(distance_m, approach_speed_mps, ttc_s, is_candidate, s):
    if not is_candidate or approach_speed_mps <= 0.0 or ttc_s is None:
        return "SAFE"

    if distance_m <= s.emergency_distance_m or ttc_s <= s.ttc_stop_s:
        return "STOP"

    if ttc_s <= s.ttc_avoid_s:
        return "AVOID"

    if ttc_s <= s.ttc_warning_s:
        return "WARNING"

    if ttc_s <= s.ttc_watch_s:
        return "WATCH"

    return "SAFE"


def calculate_point_metrics(point, s: RadarSettings):
    x = point["x"]
    y = point["y"]
    z = point["z"]
    doppler = point["doppler"]
    snr = point.get("snr")

    horizontal_distance_m = math.hypot(x, y)
    distance_m = math.sqrt(x * x + y * y + z * z)

    # Radar y-axis = forward, x-axis = left/right
    angle_deg = math.degrees(math.atan2(x, y))

    signed_approach_mps = s.approach_sign * doppler

    if signed_approach_mps > s.doppler_deadband_mps:
        approach_speed_mps = signed_approach_mps
    else:
        approach_speed_mps = 0.0

    if approach_speed_mps > 0.0:
        ttc_s = distance_m / approach_speed_mps
    else:
        ttc_s = None

    range_ok = s.min_range_m <= distance_m <= s.max_range_m
    snr_ok = snr is None or snr >= s.min_snr_db
    angle_ok = abs(angle_deg) <= s.fov_half_angle_deg
    motion_ok = approach_speed_mps > 0.0

    is_candidate = range_ok and snr_ok and angle_ok and motion_ok

    risk = classify_risk(
        distance_m,
        approach_speed_mps,
        ttc_s,
        is_candidate,
        s,
    )

    return {
        "distance_m": distance_m,
        "horizontal_distance_m": horizontal_distance_m,
        "angle_deg": angle_deg,
        "approach_speed_mps": approach_speed_mps,
        "ttc_s": ttc_s,
        "is_candidate": is_candidate,
        "risk": risk,
        "range_ok": range_ok,
        "snr_ok": snr_ok,
        "angle_ok": angle_ok,
        "motion_ok": motion_ok,
    }


# ============================================================
# Radar worker thread
# ============================================================

class RadarWorker(threading.Thread):
    def __init__(
        self,
        cli_port,
        data_port,
        cfg_path,
        settings_provider,
        event_queue,
        csv_enabled=True,
        raw_only=False,
    ):
        super().__init__(daemon=True)
        self.cli_port = cli_port
        self.data_port = data_port
        self.cfg_path = cfg_path
        self.settings_provider = settings_provider
        self.event_queue = event_queue
        self.csv_enabled = csv_enabled
        self.raw_only = raw_only

        self.stop_event = threading.Event()
        self.data_serial = None
        self.cli_serial = None
        self.csv_file = None
        self.csv_writer = None
        self.start_time = None

    def emit(self, kind, payload=None):
        self.event_queue.put((kind, payload))

    def stop(self):
        self.stop_event.set()

    def _read_cli_response(self, duration=0.10):
        if self.cli_serial is None:
            return ""

        end_t = time.time() + duration
        parts = []

        while time.time() < end_t:
            try:
                waiting = self.cli_serial.in_waiting
                if waiting:
                    parts.append(
                        self.cli_serial.read(waiting).decode(
                            errors="replace"
                        )
                    )
            except Exception:
                break
            time.sleep(0.01)

        return "".join(parts).strip()

    def configure_radar(self):
        if not os.path.isfile(self.cfg_path):
            raise FileNotFoundError(
                f"CFG file not found:\n{self.cfg_path}"
            )

        self.emit("status", "CLI 연결 중...")

        self.cli_serial = serial.Serial(
            self.cli_port,
            115200,
            timeout=0.25,
        )

        time.sleep(0.6)
        self.cli_serial.reset_input_buffer()

        self.cli_serial.write(b"sensorStop\n")
        self.cli_serial.flush()
        time.sleep(0.15)
        self._read_cli_response(0.10)

        self.emit("status", "CFG 전송 중...")

        with open(self.cfg_path, "r", encoding="utf-8", errors="ignore") as f:
            for raw_line in f:
                if self.stop_event.is_set():
                    return

                line = raw_line.strip()

                if not line or line.startswith("%"):
                    continue

                if line.startswith("sensorStart"):
                    continue

                self.cli_serial.write((line + "\n").encode())
                self.cli_serial.flush()
                time.sleep(0.03)

                response = self._read_cli_response(0.025)
                if response:
                    self.emit("cli", f"> {line}\n{response}")
                else:
                    self.emit("cli", f"> {line}")

        self.cli_serial.write(b"sensorStart\n")
        self.cli_serial.flush()
        time.sleep(0.20)

        response = self._read_cli_response(0.15)
        self.emit("cli", "> sensorStart" + (f"\n{response}" if response else ""))

        try:
            self.cli_serial.close()
        finally:
            self.cli_serial = None

    def open_csv(self):
        if not self.csv_enabled:
            return

        base_dir = os.path.dirname(os.path.abspath(self.cfg_path))
        filename = datetime.now().strftime(
            "radar_gui_log_%Y%m%d_%H%M%S.csv"
        )
        csv_path = os.path.join(base_dir, filename)

        self.csv_file = open(
            csv_path,
            "w",
            newline="",
            encoding="utf-8-sig",
        )

        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "timestamp_s",
            "frame_id",
            "point_id",
            "x_m",
            "y_m",
            "z_m",
            "distance_m",
            "horizontal_distance_m",
            "angle_deg",
            "doppler_mps",
            "approach_speed_mps",
            "ttc_s",
            "risk",
            "is_candidate",
            "snr_db",
            "noise_db",
        ])

        self.emit("csv", csv_path)

    def write_csv(self, elapsed, frame_id, processed):
        if self.csv_writer is None:
            return

        for item in processed:
            point_id = item["point_id"]
            p = item["point"]
            m = item["metrics"]

            self.csv_writer.writerow([
                round(elapsed, 3),
                frame_id,
                point_id,
                p["x"],
                p["y"],
                p["z"],
                m["distance_m"],
                m["horizontal_distance_m"],
                m["angle_deg"],
                p["doppler"],
                m["approach_speed_mps"],
                "" if m["ttc_s"] is None else m["ttc_s"],
                m["risk"],
                int(m["is_candidate"]),
                "" if p.get("snr") is None else p["snr"],
                "" if p.get("noise") is None else p["noise"],
            ])

        self.csv_file.flush()

    def write_raw_csv(self, elapsed, frame_id, points):
        if self.csv_writer is None:
            return
        for point_id, p in enumerate(points):
            try:
                x, y, z = (float(p[k]) for k in ("x", "y", "z"))
                distance = math.sqrt(x*x + y*y + z*z)
                horizontal = math.hypot(x, y)
                angle = math.degrees(math.atan2(x, y))
            except (KeyError, TypeError, ValueError):
                continue
            self.csv_writer.writerow([round(elapsed, 3), frame_id, point_id, x, y, z,
                                      distance, horizontal, angle, p.get("doppler"),
                                      "", "", "N/A", 0,
                                      "" if p.get("snr") is None else p["snr"],
                                      "" if p.get("noise") is None else p["noise"]])
        self.csv_file.flush()

    def run(self):
        try:
            self.configure_radar()

            if self.stop_event.is_set():
                return

            self.emit("status", "DATA 포트 연결 중...")

            self.data_serial = serial.Serial(
                self.data_port,
                921600,
                timeout=0.05,
            )

            time.sleep(0.3)
            self.data_serial.reset_input_buffer()

            self.open_csv()
            self.start_time = time.time()
            self.emit("status", "수신 중")
            self.emit("running", True)

            buffer = bytearray()

            while not self.stop_event.is_set():
                available = self.data_serial.in_waiting

                if available > 0:
                    buffer.extend(
                        self.data_serial.read(available)
                    )
                else:
                    time.sleep(0.002)

                while not self.stop_event.is_set():
                    magic_index = buffer.find(MAGIC_WORD)

                    if magic_index == -1:
                        if len(buffer) > 8192:
                            buffer = buffer[-7:]
                        break

                    if magic_index > 0:
                        del buffer[:magic_index]

                    if len(buffer) < HEADER_SIZE:
                        break

                    try:
                        total_packet_len = struct.unpack_from(
                            "<I", buffer, 12
                        )[0]
                    except struct.error:
                        break

                    if (
                        total_packet_len < HEADER_SIZE
                        or total_packet_len > 65536
                    ):
                        del buffer[0]
                        continue

                    if len(buffer) < total_packet_len:
                        break

                    packet = bytes(buffer[:total_packet_len])
                    del buffer[:total_packet_len]

                    result = parse_frame(packet)
                    if result is None:
                        continue

                    if self.raw_only:
                        elapsed = time.time() - self.start_time
                        self.write_raw_csv(elapsed, result["frame"], result["points"])
                        self.emit("raw_frame", (result, time.time()))
                        continue

                    settings = self.settings_provider()
                    points = result["points"]

                    processed = []

                    for point_id, p in enumerate(points):
                        metrics = calculate_point_metrics(
                            p,
                            settings,
                        )
                        processed.append({
                            "point_id": point_id,
                            "point": p,
                            "metrics": metrics,
                        })

                    candidates = [
                        item
                        for item in processed
                        if item["metrics"]["is_candidate"]
                        and item["metrics"]["ttc_s"] is not None
                    ]

                    target = (
                        min(
                            candidates,
                            key=lambda item: item["metrics"]["ttc_s"],
                        )
                        if candidates else None
                    )

                    elapsed = time.time() - self.start_time

                    self.write_csv(
                        elapsed,
                        result["frame"],
                        processed,
                    )

                    self.emit("frame", {
                        "frame_id": result["frame"],
                        "num_tlvs": result["num_tlvs"],
                        "processed": processed,
                        "candidates": candidates,
                        "target": target,
                        "settings": settings,
                    })

        except Exception as e:
            self.emit("error", f"{type(e).__name__}: {e}")

        finally:
            try:
                if self.csv_file is not None:
                    self.csv_file.close()
            except Exception:
                pass

            try:
                if self.data_serial is not None:
                    self.data_serial.close()
            except Exception:
                pass

            try:
                if self.cli_serial is not None:
                    self.cli_serial.close()
            except Exception:
                pass

            self.emit("running", False)
            self.emit("status", "정지")


# ============================================================
# GUI
# ============================================================

class RadarGUI(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("IWR6843AOP Robot Radar Monitor")
        self.geometry("1450x900")
        self.minsize(1220, 760)

        self.event_queue = queue.Queue()
        self.worker = None
        self.latest_frame = None

        self.settings_lock = threading.Lock()
        self.current_settings = RadarSettings()

        self._build_vars()
        self._build_style()
        self._build_ui()
        self.refresh_ports()
        self._guess_ports()
        self._default_cfg_path()

        self.after(50, self.process_events)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # --------------------------------------------------------
    # GUI setup
    # --------------------------------------------------------

    def _build_vars(self):
        self.cli_port_var = tk.StringVar(value="COM5")
        self.data_port_var = tk.StringVar(value="COM3")
        self.cfg_path_var = tk.StringVar()

        self.status_var = tk.StringVar(value="대기")
        self.csv_path_var = tk.StringVar(value="-")
        self.csv_enabled_var = tk.BooleanVar(value=True)
        self.show_safe_var = tk.BooleanVar(value=True)

        self.approach_sign_var = tk.StringVar(value="-1")

        s = self.current_settings

        self.deadband_var = tk.StringVar(value=f"{s.doppler_deadband_mps:.2f}")
        self.min_range_var = tk.StringVar(value=f"{s.min_range_m:.2f}")
        self.max_range_var = tk.StringVar(value=f"{s.max_range_m:.2f}")
        self.min_snr_var = tk.StringVar(value=f"{s.min_snr_db:.1f}")
        self.fov_var = tk.StringVar(value=f"{s.fov_half_angle_deg:.1f}")

        self.ttc_watch_var = tk.StringVar(value=f"{s.ttc_watch_s:.1f}")
        self.ttc_warning_var = tk.StringVar(value=f"{s.ttc_warning_s:.1f}")
        self.ttc_avoid_var = tk.StringVar(value=f"{s.ttc_avoid_s:.1f}")
        self.ttc_stop_var = tk.StringVar(value=f"{s.ttc_stop_s:.1f}")
        self.emergency_distance_var = tk.StringVar(
            value=f"{s.emergency_distance_m:.2f}"
        )

        self.frame_var = tk.StringVar(value="-")
        self.points_var = tk.StringVar(value="0")
        self.candidates_var = tk.StringVar(value="0")
        self.risk_var = tk.StringVar(value="SAFE")
        self.target_dist_var = tk.StringVar(value="-")
        self.target_speed_var = tk.StringVar(value="-")
        self.target_ttc_var = tk.StringVar(value="-")
        self.target_angle_var = tk.StringVar(value="-")
        self.target_snr_var = tk.StringVar(value="-")

    def _build_style(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(
            "Title.TLabel",
            font=("Segoe UI", 15, "bold"),
        )
        style.configure(
            "CardTitle.TLabel",
            font=("Segoe UI", 9),
        )
        style.configure(
            "CardValue.TLabel",
            font=("Segoe UI", 16, "bold"),
        )
        style.configure(
            "Small.TLabel",
            font=("Segoe UI", 9),
        )
        style.configure(
            "Treeview",
            rowheight=24,
            font=("Consolas", 9),
        )
        style.configure(
            "Treeview.Heading",
            font=("Segoe UI", 9, "bold"),
        )

    def _build_ui(self):
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=3)
        self.columnconfigure(2, weight=2)
        self.rowconfigure(0, weight=1)

        left = ttk.Frame(self, padding=10)
        center = ttk.Frame(self, padding=(0, 10, 5, 10))
        right = ttk.Frame(self, padding=(5, 10, 10, 10))

        left.grid(row=0, column=0, sticky="nsew")
        center.grid(row=0, column=1, sticky="nsew")
        right.grid(row=0, column=2, sticky="nsew")

        center.rowconfigure(1, weight=1)
        center.columnconfigure(0, weight=1)

        right.rowconfigure(2, weight=1)
        right.columnconfigure(0, weight=1)

        # LEFT ------------------------------------------------
        ttk.Label(
            left,
            text="Robot Radar Control",
            style="Title.TLabel",
        ).pack(anchor="w", pady=(0, 10))

        conn = ttk.LabelFrame(left, text="Connection", padding=8)
        conn.pack(fill="x", pady=(0, 8))

        ttk.Label(conn, text="CLI Port").grid(
            row=0, column=0, sticky="w", pady=3
        )
        self.cli_combo = ttk.Combobox(
            conn,
            textvariable=self.cli_port_var,
            width=12,
        )
        self.cli_combo.grid(row=0, column=1, sticky="ew", pady=3)

        ttk.Label(conn, text="DATA Port").grid(
            row=1, column=0, sticky="w", pady=3
        )
        self.data_combo = ttk.Combobox(
            conn,
            textvariable=self.data_port_var,
            width=12,
        )
        self.data_combo.grid(row=1, column=1, sticky="ew", pady=3)

        ttk.Button(
            conn,
            text="포트 새로고침",
            command=self.refresh_ports,
        ).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(5, 2))

        ttk.Label(conn, text="CFG").grid(
            row=3, column=0, sticky="w", pady=(8, 3)
        )
        ttk.Entry(
            conn,
            textvariable=self.cfg_path_var,
            width=27,
        ).grid(row=4, column=0, columnspan=2, sticky="ew", pady=3)

        ttk.Button(
            conn,
            text="CFG 선택",
            command=self.select_cfg,
        ).grid(row=5, column=0, columnspan=2, sticky="ew", pady=3)

        conn.columnconfigure(1, weight=1)

        filt = ttk.LabelFrame(left, text="Live Filter", padding=8)
        filt.pack(fill="x", pady=(0, 8))

        self._add_setting_row(
            filt, 0, "Doppler deadband", self.deadband_var, "m/s"
        )
        self._add_setting_row(
            filt, 1, "Min range", self.min_range_var, "m"
        )
        self._add_setting_row(
            filt, 2, "Max range", self.max_range_var, "m"
        )
        self._add_setting_row(
            filt, 3, "Min SNR", self.min_snr_var, "dB"
        )
        self._add_setting_row(
            filt, 4, "FOV ±", self.fov_var, "deg"
        )

        ttk.Label(filt, text="Approach sign").grid(
            row=5, column=0, sticky="w", pady=3
        )
        sign_box = ttk.Combobox(
            filt,
            textvariable=self.approach_sign_var,
            values=["-1", "+1"],
            state="readonly",
            width=7,
        )
        sign_box.grid(row=5, column=1, sticky="ew", pady=3)

        risk_box = ttk.LabelFrame(left, text="TTC / Risk", padding=8)
        risk_box.pack(fill="x", pady=(0, 8))

        self._add_setting_row(
            risk_box, 0, "WATCH <", self.ttc_watch_var, "s"
        )
        self._add_setting_row(
            risk_box, 1, "WARNING <", self.ttc_warning_var, "s"
        )
        self._add_setting_row(
            risk_box, 2, "AVOID <", self.ttc_avoid_var, "s"
        )
        self._add_setting_row(
            risk_box, 3, "STOP <", self.ttc_stop_var, "s"
        )
        self._add_setting_row(
            risk_box, 4, "Emergency dist", self.emergency_distance_var, "m"
        )

        ttk.Button(
            left,
            text="설정 적용",
            command=self.apply_settings,
        ).pack(fill="x", pady=(0, 5))

        ttk.Checkbutton(
            left,
            text="SAFE 포인트도 표시",
            variable=self.show_safe_var,
            command=self.redraw_latest,
        ).pack(anchor="w", pady=(2, 2))

        ttk.Checkbutton(
            left,
            text="CSV 저장",
            variable=self.csv_enabled_var,
        ).pack(anchor="w", pady=(0, 8))

        self.start_button = ttk.Button(
            left,
            text="START RADAR",
            command=self.start_radar,
        )
        self.start_button.pack(fill="x", ipady=6, pady=(2, 5))

        self.stop_button = ttk.Button(
            left,
            text="STOP",
            command=self.stop_radar,
            state="disabled",
        )
        self.stop_button.pack(fill="x", ipady=4, pady=(0, 8))

        status_box = ttk.LabelFrame(left, text="Status", padding=8)
        status_box.pack(fill="both", expand=True)

        ttk.Label(
            status_box,
            textvariable=self.status_var,
            font=("Segoe UI", 10, "bold"),
        ).pack(anchor="w")

        ttk.Label(
            status_box,
            text="CSV:",
            style="Small.TLabel",
        ).pack(anchor="w", pady=(8, 0))

        ttk.Label(
            status_box,
            textvariable=self.csv_path_var,
            wraplength=260,
            style="Small.TLabel",
        ).pack(anchor="w")

        # CENTER ----------------------------------------------
        topbar = ttk.Frame(center)
        topbar.grid(row=0, column=0, sticky="ew", pady=(0, 6))

        ttk.Label(
            topbar,
            text="Top-down Radar View",
            style="Title.TLabel",
        ).pack(side="left")

        ttk.Label(
            topbar,
            text="  y = forward / x = left-right",
            style="Small.TLabel",
        ).pack(side="left")

        self.canvas = tk.Canvas(
            center,
            background="#111820",
            highlightthickness=1,
            highlightbackground="#46515c",
        )
        self.canvas.grid(row=1, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", lambda e: self.redraw_latest())

        legend = ttk.Frame(center)
        legend.grid(row=2, column=0, sticky="ew", pady=(6, 0))

        for text_label, color in [
            ("SAFE", "#9aa4ad"),
            ("WATCH", "#67b7dc"),
            ("WARNING", "#f0c64e"),
            ("AVOID", "#f28e2b"),
            ("STOP", "#e15759"),
            ("Target", "#ffffff"),
        ]:
            sw = tk.Canvas(
                legend,
                width=12,
                height=12,
                highlightthickness=0,
            )
            sw.create_oval(2, 2, 10, 10, fill=color, outline=color)
            sw.pack(side="left", padx=(0, 3))
            ttk.Label(legend, text=text_label).pack(
                side="left", padx=(0, 12)
            )

        # RIGHT -----------------------------------------------
        cards = ttk.LabelFrame(right, text="Current Target", padding=8)
        cards.grid(row=0, column=0, sticky="ew")

        cards.columnconfigure((0, 1, 2), weight=1)

        self._make_card(cards, 0, 0, "FRAME", self.frame_var)
        self._make_card(cards, 0, 1, "POINTS", self.points_var)
        self._make_card(cards, 0, 2, "CANDIDATES", self.candidates_var)

        self.risk_card = self._make_card(
            cards, 1, 0, "RISK", self.risk_var
        )
        self._make_card(cards, 1, 1, "DISTANCE", self.target_dist_var)
        self._make_card(cards, 1, 2, "APPROACH", self.target_speed_var)

        self._make_card(cards, 2, 0, "TTC", self.target_ttc_var)
        self._make_card(cards, 2, 1, "ANGLE", self.target_angle_var)
        self._make_card(cards, 2, 2, "SNR", self.target_snr_var)

        log_frame = ttk.LabelFrame(right, text="CLI / Event Log", padding=5)
        log_frame.grid(row=1, column=0, sticky="ew", pady=(8, 8))

        self.log_text = tk.Text(
            log_frame,
            height=8,
            wrap="word",
            font=("Consolas", 8),
        )
        self.log_text.pack(fill="both", expand=True)

        table_frame = ttk.LabelFrame(right, text="Points", padding=5)
        table_frame.grid(row=2, column=0, sticky="nsew")
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)

        columns = (
            "id",
            "dist",
            "angle",
            "dop",
            "approach",
            "ttc",
            "snr",
            "risk",
        )

        self.tree = ttk.Treeview(
            table_frame,
            columns=columns,
            show="headings",
        )

        headers = {
            "id": ("ID", 42),
            "dist": ("Dist", 62),
            "angle": ("Angle", 67),
            "dop": ("Dop", 62),
            "approach": ("Approach", 75),
            "ttc": ("TTC", 60),
            "snr": ("SNR", 58),
            "risk": ("Risk", 70),
        }

        for c, (title, width) in headers.items():
            self.tree.heading(c, text=title)
            self.tree.column(c, width=width, anchor="center")

        self.tree.grid(row=0, column=0, sticky="nsew")

        yscroll = ttk.Scrollbar(
            table_frame,
            orient="vertical",
            command=self.tree.yview,
        )
        yscroll.grid(row=0, column=1, sticky="ns")
        self.tree.configure(yscrollcommand=yscroll.set)

    def _add_setting_row(self, parent, row, label, var, unit):
        ttk.Label(parent, text=label).grid(
            row=row, column=0, sticky="w", pady=3
        )

        ttk.Entry(
            parent,
            textvariable=var,
            width=8,
        ).grid(row=row, column=1, sticky="ew", padx=(6, 3), pady=3)

        ttk.Label(parent, text=unit).grid(
            row=row, column=2, sticky="w", pady=3
        )

        parent.columnconfigure(1, weight=1)

    def _make_card(self, parent, row, col, title, variable):
        frame = ttk.Frame(parent, padding=(6, 6))
        frame.grid(row=row, column=col, sticky="nsew", padx=3, pady=3)

        ttk.Label(
            frame,
            text=title,
            style="CardTitle.TLabel",
        ).pack()

        value_label = ttk.Label(
            frame,
            textvariable=variable,
            style="CardValue.TLabel",
        )
        value_label.pack()

        return value_label

    # --------------------------------------------------------
    # Port / CFG
    # --------------------------------------------------------

    def refresh_ports(self):
        ports = list(list_ports.comports())

        values = [p.device for p in ports]

        self.cli_combo["values"] = values
        self.data_combo["values"] = values

        self._append_log("Detected ports:")
        for p in ports:
            self._append_log(f"  {p.device} | {p.description}")

        self._guess_ports()

    def _guess_ports(self):
        try:
            ports = list(list_ports.comports())

            enhanced = None
            standard = None

            for p in ports:
                desc = (p.description or "").lower()

                if "enhanced" in desc:
                    enhanced = p.device

                if "standard" in desc:
                    standard = p.device

            if enhanced:
                self.cli_port_var.set(enhanced)

            if standard:
                self.data_port_var.set(standard)

        except Exception:
            pass

    def _default_cfg_path(self):
        candidates = [
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "profile_3d_aop.cfg",
            ),
            os.path.join(
                os.path.expanduser("~"),
                "Downloads",
                "profile_3d_aop.cfg",
            ),
        ]

        for path in candidates:
            if os.path.isfile(path):
                self.cfg_path_var.set(path)
                return

        self.cfg_path_var.set(candidates[0])

    def select_cfg(self):
        path = filedialog.askopenfilename(
            title="Select radar cfg",
            filetypes=[
                ("Radar cfg", "*.cfg"),
                ("All files", "*.*"),
            ],
        )

        if path:
            self.cfg_path_var.set(path)

    # --------------------------------------------------------
    # Settings
    # --------------------------------------------------------

    def apply_settings(self):
        try:
            new_s = RadarSettings(
                approach_sign=float(self.approach_sign_var.get()),
                doppler_deadband_mps=float(self.deadband_var.get()),
                min_range_m=float(self.min_range_var.get()),
                max_range_m=float(self.max_range_var.get()),
                min_snr_db=float(self.min_snr_var.get()),
                fov_half_angle_deg=float(self.fov_var.get()),
                ttc_watch_s=float(self.ttc_watch_var.get()),
                ttc_warning_s=float(self.ttc_warning_var.get()),
                ttc_avoid_s=float(self.ttc_avoid_var.get()),
                ttc_stop_s=float(self.ttc_stop_var.get()),
                emergency_distance_m=float(
                    self.emergency_distance_var.get()
                ),
            )

            if new_s.min_range_m < 0:
                raise ValueError("Min range must be >= 0.")

            if new_s.max_range_m <= new_s.min_range_m:
                raise ValueError("Max range must be greater than Min range.")

            if not (
                new_s.ttc_stop_s
                <= new_s.ttc_avoid_s
                <= new_s.ttc_warning_s
                <= new_s.ttc_watch_s
            ):
                raise ValueError(
                    "TTC must satisfy STOP <= AVOID <= WARNING <= WATCH."
                )

            if not (0.0 < new_s.fov_half_angle_deg <= 180.0):
                raise ValueError("FOV must be between 0 and 180 degrees.")

            with self.settings_lock:
                self.current_settings = new_s

            self.status_var.set("설정 적용됨")
            self._append_log(
                "Filter updated: "
                f"range={new_s.min_range_m:.2f}~{new_s.max_range_m:.2f}m, "
                f"SNR>={new_s.min_snr_db:.1f}dB, "
                f"deadband={new_s.doppler_deadband_mps:.2f}m/s, "
                f"FOV=±{new_s.fov_half_angle_deg:.0f}°"
            )

            self.redraw_latest()

        except Exception as e:
            messagebox.showerror("설정 오류", str(e))

    def get_settings_snapshot(self):
        with self.settings_lock:
            s = self.current_settings

            return RadarSettings(
                approach_sign=s.approach_sign,
                doppler_deadband_mps=s.doppler_deadband_mps,
                min_range_m=s.min_range_m,
                max_range_m=s.max_range_m,
                min_snr_db=s.min_snr_db,
                fov_half_angle_deg=s.fov_half_angle_deg,
                ttc_watch_s=s.ttc_watch_s,
                ttc_warning_s=s.ttc_warning_s,
                ttc_avoid_s=s.ttc_avoid_s,
                ttc_stop_s=s.ttc_stop_s,
                emergency_distance_m=s.emergency_distance_m,
            )

    # --------------------------------------------------------
    # Start / stop
    # --------------------------------------------------------

    def start_radar(self):
        if self.worker is not None and self.worker.is_alive():
            return

        self.apply_settings()

        cfg_path = self.cfg_path_var.get().strip()

        if not os.path.isfile(cfg_path):
            messagebox.showerror(
                "CFG 없음",
                f"CFG 파일을 찾을 수 없습니다.\n\n{cfg_path}",
            )
            return

        cli_port = self.cli_port_var.get().strip()
        data_port = self.data_port_var.get().strip()

        if not cli_port or not data_port:
            messagebox.showerror(
                "포트 오류",
                "CLI / DATA 포트를 선택하세요.",
            )
            return

        if cli_port == data_port:
            messagebox.showerror(
                "포트 오류",
                "CLI와 DATA 포트는 서로 달라야 합니다.",
            )
            return

        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")

        self.worker = RadarWorker(
            cli_port=cli_port,
            data_port=data_port,
            cfg_path=cfg_path,
            settings_provider=self.get_settings_snapshot,
            event_queue=self.event_queue,
            csv_enabled=self.csv_enabled_var.get(),
        )

        self.worker.start()

    def stop_radar(self):
        if self.worker is not None:
            self.status_var.set("정지 요청 중...")
            self.worker.stop()

    # --------------------------------------------------------
    # Events
    # --------------------------------------------------------

    def process_events(self):
        processed_count = 0

        while processed_count < 100:
            try:
                kind, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break

            processed_count += 1

            if kind == "status":
                self.status_var.set(str(payload))

            elif kind == "cli":
                self._append_log(str(payload))

            elif kind == "csv":
                self.csv_path_var.set(str(payload))
                self._append_log(f"CSV: {payload}")

            elif kind == "running":
                running = bool(payload)

                self.start_button.configure(
                    state="disabled" if running else "normal"
                )
                self.stop_button.configure(
                    state="normal" if running else "disabled"
                )

            elif kind == "error":
                self._append_log(f"ERROR: {payload}")
                self.status_var.set("오류")

                self.start_button.configure(state="normal")
                self.stop_button.configure(state="disabled")

                messagebox.showerror("Radar error", str(payload))

            elif kind == "frame":
                self.latest_frame = payload
                self.update_frame(payload)

        self.after(50, self.process_events)

    # --------------------------------------------------------
    # Frame display
    # --------------------------------------------------------

    def update_frame(self, frame):
        processed = frame["processed"]
        candidates = frame["candidates"]
        target = frame["target"]

        self.frame_var.set(str(frame["frame_id"]))
        self.points_var.set(str(len(processed)))
        self.candidates_var.set(str(len(candidates)))

        if target is None:
            self.risk_var.set("SAFE")
            self.target_dist_var.set("-")
            self.target_speed_var.set("-")
            self.target_ttc_var.set("-")
            self.target_angle_var.set("-")
            self.target_snr_var.set("-")
        else:
            p = target["point"]
            m = target["metrics"]

            self.risk_var.set(m["risk"])
            self.target_dist_var.set(f"{m['distance_m']:.2f} m")
            self.target_speed_var.set(
                f"{m['approach_speed_mps']:.2f} m/s"
            )
            self.target_ttc_var.set(f"{m['ttc_s']:.2f} s")
            self.target_angle_var.set(f"{m['angle_deg']:+.1f}°")

            snr = p.get("snr")
            self.target_snr_var.set(
                "-" if snr is None else f"{snr:.1f} dB"
            )

        self.update_table(processed)
        self.draw_radar(frame)

    def update_table(self, processed):
        for item in self.tree.get_children():
            self.tree.delete(item)

        show_safe = self.show_safe_var.get()

        visible = []

        for item in processed:
            m = item["metrics"]

            if not show_safe and not m["is_candidate"]:
                continue

            visible.append(item)

        def sort_key(item):
            m = item["metrics"]
            ttc = m["ttc_s"]

            if m["is_candidate"] and ttc is not None:
                return (0, ttc)

            return (1, m["distance_m"])

        visible.sort(key=sort_key)

        for item in visible[:120]:
            pid = item["point_id"]
            p = item["point"]
            m = item["metrics"]

            ttc_text = (
                "-"
                if m["ttc_s"] is None
                else f"{m['ttc_s']:.2f}"
            )

            snr = p.get("snr")
            snr_text = "-" if snr is None else f"{snr:.1f}"

            self.tree.insert(
                "",
                "end",
                values=(
                    pid,
                    f"{m['distance_m']:.2f}",
                    f"{m['angle_deg']:+.1f}",
                    f"{p['doppler']:+.2f}",
                    f"{m['approach_speed_mps']:.2f}",
                    ttc_text,
                    snr_text,
                    m["risk"],
                ),
            )

    def redraw_latest(self):
        if self.latest_frame is not None:
            self.update_table(self.latest_frame["processed"])
            self.draw_radar(self.latest_frame)
        else:
            self.draw_empty_radar()

    def draw_empty_radar(self):
        self.canvas.delete("all")

        w = max(self.canvas.winfo_width(), 300)
        h = max(self.canvas.winfo_height(), 300)

        cx = w / 2
        origin_y = h - 45

        self.canvas.create_oval(
            cx - 8,
            origin_y - 8,
            cx + 8,
            origin_y + 8,
            fill="#ffffff",
            outline="",
        )

        self.canvas.create_text(
            cx,
            origin_y + 20,
            text="ROBOT / RADAR",
            fill="#cbd5df",
            font=("Segoe UI", 9, "bold"),
        )

    def draw_radar(self, frame):
        self.canvas.delete("all")

        w = max(self.canvas.winfo_width(), 400)
        h = max(self.canvas.winfo_height(), 400)

        cx = w / 2
        origin_y = h - 45

        settings = frame["settings"]
        plot_range = max(settings.max_range_m, 0.5)

        usable_h = max(h - 80, 100)
        usable_w_half = max(w / 2 - 35, 100)

        scale = min(
            usable_h / plot_range,
            usable_w_half / plot_range,
        )

        # Range rings
        ring_steps = 4
        for i in range(1, ring_steps + 1):
            r_m = plot_range * i / ring_steps
            r_px = r_m * scale

            self.canvas.create_arc(
                cx - r_px,
                origin_y - r_px,
                cx + r_px,
                origin_y + r_px,
                start=0,
                extent=180,
                style="arc",
                outline="#33404c",
                width=1,
            )

            self.canvas.create_text(
                cx + 5,
                origin_y - r_px,
                text=f"{r_m:.2f}m",
                fill="#7f8c98",
                anchor="sw",
                font=("Segoe UI", 8),
            )

        # Center line
        self.canvas.create_line(
            cx,
            origin_y,
            cx,
            20,
            fill="#3c4c59",
            dash=(4, 4),
        )

        # FOV lines
        fov_rad = math.radians(settings.fov_half_angle_deg)

        for sign in (-1, +1):
            dx = math.sin(fov_rad) * plot_range * scale * sign
            dy = math.cos(fov_rad) * plot_range * scale

            self.canvas.create_line(
                cx,
                origin_y,
                cx + dx,
                origin_y - dy,
                fill="#40576a",
                dash=(3, 4),
            )

        # Robot
        self.canvas.create_polygon(
            cx,
            origin_y - 12,
            cx - 11,
            origin_y + 9,
            cx + 11,
            origin_y + 9,
            fill="#ffffff",
            outline="",
        )

        self.canvas.create_text(
            cx,
            origin_y + 22,
            text="ROBOT",
            fill="#cbd5df",
            font=("Segoe UI", 9, "bold"),
        )

        target = frame["target"]
        target_id = target["point_id"] if target else None

        risk_colors = {
            "SAFE": "#9aa4ad",
            "WATCH": "#67b7dc",
            "WARNING": "#f0c64e",
            "AVOID": "#f28e2b",
            "STOP": "#e15759",
        }

        show_safe = self.show_safe_var.get()

        for item in frame["processed"]:
            pid = item["point_id"]
            p = item["point"]
            m = item["metrics"]

            if not show_safe and not m["is_candidate"]:
                continue

            # Only plot the forward half-plane.
            # Points with y < 0 are behind the radar and are skipped in this view.
            if p["y"] < 0:
                continue

            px = cx + p["x"] * scale
            py = origin_y - p["y"] * scale

            if not (10 <= px <= w - 10 and 10 <= py <= h - 10):
                continue

            color = risk_colors.get(m["risk"], "#9aa4ad")

            radius = 7 if m["is_candidate"] else 4

            if pid == target_id:
                self.canvas.create_oval(
                    px - 11,
                    py - 11,
                    px + 11,
                    py + 11,
                    outline="#ffffff",
                    width=2,
                )
                radius = 8

            self.canvas.create_oval(
                px - radius,
                py - radius,
                px + radius,
                py + radius,
                fill=color,
                outline="",
            )

            if m["is_candidate"] or pid == target_id:
                label = (
                    f"P{pid} "
                    f"{m['distance_m']:.2f}m "
                    f"{m['approach_speed_mps']:.2f}m/s"
                )

                self.canvas.create_text(
                    px + 9,
                    py - 9,
                    text=label,
                    fill="#e8eef4",
                    anchor="sw",
                    font=("Consolas", 8),
                )

    # --------------------------------------------------------
    # Utility
    # --------------------------------------------------------

    def _append_log(self, text):
        if not hasattr(self, "log_text"):
            return

        self.log_text.insert("end", str(text) + "\n")
        self.log_text.see("end")

        # Prevent unlimited text growth
        try:
            line_count = int(self.log_text.index("end-1c").split(".")[0])
            if line_count > 800:
                self.log_text.delete("1.0", "200.0")
        except Exception:
            pass

    def on_close(self):
        if self.worker is not None:
            self.worker.stop()

        self.after(100, self.destroy)


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":
    app = RadarGUI()
    app.mainloop()
