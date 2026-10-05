"""Object-oriented live/replay GUI. Run with: python radar_gui.py"""
import csv
from dataclasses import asdict, fields
from datetime import datetime
import math
import os
import queue
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from serial.tools import list_ports
from radar_gui_v1_ready import RadarSettings, RadarWorker
from radar_processing import RadarProcessor, Settings
from grise_radar_gate import GateSettings


OBJECT_FIELDS = ["timestamp", "frame", "robot_state", "object_id", "confirmed",
                 "persistence", "confidence", "motion_state", "direction_state",
                 "point_count", "centroid_x", "centroid_y", "centroid_z", "distance",
                 "angle", "doppler_velocity", "range_rate_velocity", "ttc",
                 "ttc_velocity_source", "risk", "target_selected"]


def approach_label(target, threshold_mps):
    """Only a confirmed, currently approaching range-rate gets a speed label."""
    if not target or not target.get("confirmed") or target.get("confidence") != "HIGH":
        return "UNKNOWN"
    if target.get("direction_state") != "APPROACHING":
        return "UNKNOWN"
    speed = target.get("range_rate_velocity")
    if not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed <= 0:
        return "UNKNOWN"
    return "FAST" if speed >= threshold_mps else "SLOW"


def load_replay(path):
    """Preserve frame order, including frames with no logged points when gaps exist."""
    frames = []
    with open(path, newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {"timestamp_s", "frame_id", "x_m", "y_m", "z_m", "doppler_mps"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("Unsupported CSV: missing raw point columns")
        current = None
        for row in reader:
            try:
                fid = int(row["frame_id"])
                ts = float(row["timestamp_s"])
                point = dict(x=float(row["x_m"]), y=float(row["y_m"]), z=float(row["z_m"]),
                             doppler=float(row["doppler_mps"]),
                             snr=None if not row.get("snr_db") else float(row["snr_db"]),
                             noise=None if not row.get("noise_db") else float(row["noise_db"]))
            except (TypeError, ValueError, KeyError):
                continue
            if current is None or fid != current[0]:
                current = (fid, ts, [])
                frames.append(current)
            current[2].append(point)
    return frames


class RadarApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("IWR6843AOP Object Monitor")
        self.geometry("1420x880")
        self.processor = RadarProcessor()
        self.events = queue.Queue()
        self.worker = None
        self.mode = "LIVE"
        self.replay = []
        self.replay_index = 0
        self.replay_playing = False
        self.replay_generation = 0
        self.replay_clock = None
        self.last_result = None
        self.log_handle = None
        self.object_writer = None
        self.settings_vars = {}
        self.fast_threshold_mps = GateSettings().approach_speed_mps
        self.layer_vars = {}
        self._build()
        self.after(50, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _build(self):
        root = ttk.Frame(self, padding=8)
        root.pack(fill="both", expand=True)
        left = ttk.Frame(root, width=260)
        left.pack(side="left", fill="y", padx=(0, 8))
        center = ttk.Frame(root)
        center.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(root, width=430)
        right.pack(side="left", fill="y", padx=(8, 0))

        connection = ttk.LabelFrame(left, text="Live connection", padding=6)
        connection.pack(fill="x")
        ports = [p.device for p in list_ports.comports()]
        self.cli_var = tk.StringVar(value="COM5")
        self.data_var = tk.StringVar(value="COM3")
        self.cfg_var = tk.StringVar(value=os.path.join(os.path.dirname(__file__), "profile_3d_aop.cfg"))
        for label, var in (("CLI", self.cli_var), ("DATA", self.data_var)):
            ttk.Label(connection, text=label).pack(anchor="w")
            ttk.Combobox(connection, textvariable=var, values=ports).pack(fill="x")
        ttk.Label(connection, text="CFG (OFF or experimental ON)").pack(anchor="w")
        ttk.Entry(connection, textvariable=self.cfg_var).pack(fill="x")
        ttk.Button(connection, text="Select CFG", command=self._choose_cfg).pack(fill="x")
        ttk.Button(connection, text="START LIVE", command=self._start_live).pack(fill="x")
        ttk.Button(connection, text="STOP LIVE", command=self._stop_live).pack(fill="x")
        self.raw_log_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(connection, text="Save every raw point to CSV (large)",
                        variable=self.raw_log_var).pack(anchor="w")

        replay = ttk.LabelFrame(left, text="CSV replay", padding=6)
        replay.pack(fill="x", pady=6)
        self.replay_path_var = tk.StringVar(value="-")
        ttk.Label(replay, textvariable=self.replay_path_var, wraplength=240).pack(anchor="w")
        ttk.Button(replay, text="Select CSV", command=self._choose_csv).pack(fill="x")
        controls = ttk.Frame(replay)
        controls.pack(fill="x")
        for label, action in (("PLAY", self._play), ("PAUSE", self._pause), ("STOP", self._stop_replay)):
            ttk.Button(controls, text=label, command=action).pack(side="left", expand=True, fill="x")
        self.speed_var = tk.StringVar(value="1x")
        ttk.Combobox(replay, textvariable=self.speed_var, values=("0.5x", "1x", "2x", "MAX"), state="readonly").pack(fill="x")

        robot = ttk.LabelFrame(left, text="Robot state simulation", padding=6)
        robot.pack(fill="x", pady=6)
        ttk.Button(robot, text="ROBOT STOPPED", command=lambda: self._set_moving(False)).pack(fill="x")
        ttk.Button(robot, text="ROBOT MOVING", command=lambda: self._set_moving(True)).pack(fill="x")
        self.state_var = tk.StringVar(value="MONITORING / ACTIVE")
        ttk.Label(robot, textvariable=self.state_var, foreground="#b45309").pack(anchor="w")

        setting_frame = ttk.LabelFrame(left, text="Parameters (Apply)", padding=6)
        setting_frame.pack(fill="both", expand=True)
        scroll = tk.Canvas(setting_frame, width=245, highlightthickness=0)
        bar = ttk.Scrollbar(setting_frame, orient="vertical", command=scroll.yview)
        inside = ttk.Frame(scroll)
        inside.bind("<Configure>", lambda e: scroll.configure(scrollregion=scroll.bbox("all")))
        scroll.create_window((0, 0), window=inside, anchor="nw")
        scroll.configure(yscrollcommand=bar.set)
        scroll.pack(side="left", fill="both", expand=True)
        bar.pack(side="right", fill="y")
        for f in fields(Settings):
            # Experimental Fast Path is configured through the Pi CLI; retain
            # the existing Windows settings layout and two-frame baseline flag.
            if f.name.startswith("fast_") and f.name != "fast_confidence_on_two_frames":
                continue
            var = tk.StringVar(value=str(getattr(self.processor.settings, f.name)))
            self.settings_vars[f.name] = var
            row = ttk.Frame(inside)
            row.pack(fill="x")
            ttk.Label(row, text=f.name, width=24).pack(side="left")
            ttk.Entry(row, textvariable=var, width=7).pack(side="right")
        ttk.Button(inside, text="Apply", command=self._apply).pack(fill="x")
        ttk.Label(left, text="Fast approach threshold (m/s)").pack(anchor="w")
        self.fast_threshold_var = tk.StringVar(value=str(self.fast_threshold_mps))
        ttk.Entry(left, textvariable=self.fast_threshold_var).pack(fill="x")
        ttk.Button(left, text="Apply speed threshold", command=self._apply).pack(fill="x")

        self.counter_var = tk.StringVar(value="RAW 0  ROI 0  STATIC 0  MOVING 0  APPROACHING 0  RECEDING 0  CLUSTERS 0  CONFIRMED 0  THREATS 0")
        ttk.Label(center, textvariable=self.counter_var, wraplength=680, font=("Consolas", 10, "bold")).pack(fill="x", pady=5)
        self.canvas = tk.Canvas(center, background="#111820")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self._draw())
        layers = ttk.LabelFrame(center, text="Map layers", padding=4)
        layers.pack(fill="x")
        for name, default in (("Raw", False), ("ROI", False), ("Static", False), ("Moving", False), ("Approaching", False), ("Clusters", False), ("Confirmed", True)):
            var = tk.BooleanVar(value=default)
            self.layer_vars[name] = var
            ttk.Checkbutton(layers, text=name, variable=var, command=self._draw).pack(side="left")

        self.target_var = tk.StringVar(value="Target: None | Speed: UNKNOWN")
        ttk.Label(right, textvariable=self.target_var, wraplength=420, font=("Consolas", 13, "bold")).pack(fill="x", pady=8)
        self.detail = tk.Text(right, width=48, height=14, font=("Consolas", 10))
        self.detail.pack(fill="x")
        self.table = ttk.Treeview(right, columns=("id", "dist", "angle", "dop", "range", "persist", "conf", "ttc", "risk"), show="headings", height=18)
        for col in self.table["columns"]:
            self.table.heading(col, text=col)
            self.table.column(col, width=54, anchor="center")
        self.table.pack(fill="both", expand=True)
        self.show_tentative_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(right, text="Show tentative objects (diagnostic)",
                        variable=self.show_tentative_var,
                        command=lambda: self._render(self.last_result) if self.last_result else None).pack(anchor="w")
        self.status_var = tk.StringVar(value="LIVE ready")
        ttk.Label(right, textvariable=self.status_var, wraplength=420).pack(fill="x")

    def _choose_cfg(self):
        path = filedialog.askopenfilename(filetypes=(("Radar CFG", "*.cfg"),))
        if path:
            self.cfg_var.set(path)

    def _apply(self):
        try:
            values = dict(vars(self.processor.settings))
            integer = {"approach_sign", "min_cluster_points", "temporal_window", "temporal_required", "target_release_misses"}
            boolean = {"require_snr", "diagnostic_only", "target_requires_threat", "cluster_before_direction", "fast_confidence_on_two_frames"}
            for name, var in self.settings_vars.items():
                if name in boolean:
                    if var.get().lower() not in ("true", "false"):
                        raise ValueError(f"{name} must be True or False")
                    values[name] = var.get().lower() == "true"
                else:
                    values[name] = int(var.get()) if name in integer else float(var.get())
            settings = Settings(**values)
            settings.validate()
            fast_threshold = float(self.fast_threshold_var.get())
            if not math.isfinite(fast_threshold) or fast_threshold <= 0:
                raise ValueError("Fast approach threshold must be a positive finite speed")
            self.processor.settings = settings
            self.fast_threshold_mps = fast_threshold
            self.processor.reset()
            if self.mode == "CSV REPLAY" and self.replay:
                index = self.replay_index
                self.processor.reset()
                for fid, ts, points in self.replay[:index]:
                    self.last_result = self.processor.process_frame(fid, points, ts)
                if self.last_result:
                    self._render(self.last_result)
            self.status_var.set("Parameters applied; track history rebuilt")
            return True
        except (ValueError, TypeError) as exc:
            messagebox.showerror("Parameter error", str(exc))
            return False

    def _start_live(self):
        if self.worker and self.worker.is_alive():
            return
        if not self._apply():
            return
        if not os.path.isfile(self.cfg_var.get()):
            messagebox.showerror("CFG", "Select an existing CFG file")
            return
        if not self.cli_var.get() or not self.data_var.get() or self.cli_var.get() == self.data_var.get():
            messagebox.showerror("Ports", "Select distinct CLI and DATA ports")
            return
        self._stop_replay()
        self.mode = "LIVE"
        self.processor.reset()
        self._open_object_log()
        self.worker = RadarWorker(self.cli_var.get(), self.data_var.get(), self.cfg_var.get(),
                                  lambda: RadarSettings(), self.events,
                                  csv_enabled=self.raw_log_var.get(), raw_only=True)
        self.worker.start()
        self.status_var.set("LIVE starting")

    def _stop_live(self):
        if self.worker:
            self.worker.stop()
        self._close_object_log()

    def _choose_csv(self):
        path = filedialog.askopenfilename(filetypes=(("Raw radar CSV", "*.csv"),))
        if not path:
            return
        try:
            frames = load_replay(path)
            if not frames:
                raise ValueError("No usable frames")
        except (OSError, ValueError) as exc:
            messagebox.showerror("CSV replay", str(exc))
            return
        self._stop_live()
        self.replay = frames
        self.replay_index = 0
        self.replay_playing = False
        self.replay_generation += 1
        self.replay_clock = None
        self.mode = "CSV REPLAY"
        self.processor.reset()
        self.replay_path_var.set(path)
        self.status_var.set(f"CSV REPLAY: {len(frames)} frames loaded")

    def _play(self):
        if self.replay and not self.replay_playing:
            self.replay_playing = True
            self.replay_generation += 1
            self._replay_tick(self.replay_generation)

    def _pause(self):
        self.replay_playing = False
        self.replay_generation += 1

    def _stop_replay(self):
        self.replay_playing = False
        self.replay_generation += 1
        self.replay_index = 0
        self.replay_clock = None
        self.processor.reset()
        if self.mode == "CSV REPLAY":
            self.last_result = None
            self._draw()

    def _replay_tick(self, generation):
        if generation != self.replay_generation or not self.replay_playing:
            return
        if self.replay_index >= len(self.replay):
            self.replay_playing = False
            return
        fid, ts, points = self.replay[self.replay_index]
        self.replay_index += 1
        self._process_frame(fid, points, ts)
        if self.replay_index < len(self.replay):
            next_ts = self.replay[self.replay_index][1]
            scale = {"0.5x": 0.5, "1x": 1.0, "2x": 2.0, "MAX": float("inf")}[self.speed_var.get()]
            delay = 1 if math.isinf(scale) else max(1, int(1000 * max(0, next_ts-ts) / scale))
            self.after(delay, self._replay_tick, generation)

    def _set_moving(self, moving):
        now = time.time() if self.mode == "LIVE" else (self.last_result["timestamp"] if self.last_result else 0.0)
        self.processor.set_moving(moving, now)
        self._render(self.processor.process_frame(self.last_result["frame_id"] + 1 if self.last_result else 0, [], now))

    def _poll(self):
        for _ in range(100):
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "raw_frame" and self.mode == "LIVE":
                frame, ts = payload
                self._process_frame(frame["frame"], frame["points"], ts)
            elif kind == "error":
                self.status_var.set(str(payload))
            elif kind == "status" and self.mode == "LIVE":
                self.status_var.set(str(payload))
        self.after(50, self._poll)

    def _process_frame(self, fid, points, ts):
        result = self.processor.process_frame(fid, points, ts)
        self._render(result)
        if self.mode == "LIVE" and self.object_writer:
            for obj in result["objects"]:
                x, y, z = obj["centroid"]
                self.object_writer.writerow([ts, fid, result["robot_state"], obj["id"], int(obj["confirmed"]),
                                            obj["persistence"], obj["confidence"], obj["motion_state"], obj["direction_state"],
                                            obj["point_count"], x, y, z, obj["distance"], obj["angle"], obj["doppler_velocity"],
                                            obj["range_rate_velocity"], obj["ttc"], obj["ttc_velocity_source"], obj["risk"],
                                            int(obj["target_selected"])])
            self.log_handle.flush()

    def _render(self, result):
        self.last_result = result
        counts = result["counts"]
        self.counter_var.set("  ".join(f"{key.upper()} {counts[key]}" for key in ("raw", "roi", "static", "moving", "approaching", "receding", "clusters", "tentative", "confirmed", "targets")))
        self.state_var.set(f"{result['robot_state']} / {result['processing']}")
        target = result["target"]
        ttc_text = "N/A" if target is None or target["ttc"] is None else f"{target['ttc']:.2f}s"
        speed_label = approach_label(target, self.fast_threshold_mps) if result["processing"] == "ACTIVE" else "UNKNOWN"
        speed = target.get("range_rate_velocity") if target else None
        speed_text = f"{speed:.2f}m/s" if speed_label != "UNKNOWN" else "N/A"
        self.target_var.set("Target: None | Speed: UNKNOWN | Threat: Disabled" if result["processing"] == "PAUSED" else
                            "Target: None | Speed: UNKNOWN" if target is None else
                            f"Target OBJ {target['id']} | Speed: {speed_label} ({speed_text}) | {target['risk']} | {target['distance']:.2f}m | TTC {ttc_text}")
        for item in self.table.get_children():
            self.table.delete(item)
        visible_objects = [obj for obj in result["objects"]
                           if self.show_tentative_var.get() or obj["confirmed"] and obj["point_count"] > 0]
        for obj in visible_objects:
            self.table.insert("", "end", values=(obj["id"], f"{obj['distance']:.2f}", f"{obj['angle']:+.0f}",
                f"{obj['doppler_velocity']:.2f}", "-" if obj["range_rate_velocity"] is None else f"{obj['range_rate_velocity']:.2f}",
                f"{obj['persistence']}/{self.processor.settings.temporal_window}", obj["confidence"],
                "-" if obj["ttc"] is None else f"{obj['ttc']:.2f}", obj["risk"]))
        self.detail.delete("1.0", "end")
        for obj in visible_objects:
            self.detail.insert("end", f"OBJ {obj['id']} {obj['motion_state']}/{obj['direction_state']}  {obj['confidence']}\n"
                               f"  Dist {obj['distance']:.2f}m Angle {obj['angle']:+.1f}° Points {obj['point_count']}\n"
                               f"  Dop {obj['doppler_velocity']:.2f}m/s Range {obj['range_rate_velocity']}m/s\n"
                               f"  Persist {obj['persistence']}/{self.processor.settings.temporal_window} TTC {obj['ttc']} ({obj['ttc_velocity_source']}) Risk {obj['risk']}\n"
                               f"  Raw dist {obj['raw_distance']:.2f}m Approach ratio {obj.get('approaching_point_ratio', 0):.2f} Pending {obj.get('pending_reason', '-')}\n")
        self._draw()

    def _draw(self):
        c = self.canvas
        c.delete("all")
        w, h = max(c.winfo_width(), 300), max(c.winfo_height(), 300)
        cx, oy = w / 2, h - 40
        scale = min((h-70)/self.processor.settings.max_range_m, (w/2-30)/self.processor.settings.max_range_m)
        c.create_line(cx, oy, cx, 20, fill="#4b5563", dash=(3, 3))
        c.create_oval(cx-7, oy-7, cx+7, oy+7, fill="white")
        if not self.last_result:
            return
        r = self.last_result
        layers = (("Raw", r["raw"], "#637181"), ("ROI", r["roi"], "#7d91ad"),
                  ("Static", [p for p in r["roi"] if p["motion_state"] == "STATIC"], "#888888"),
                  ("Moving", [p for p in r["roi"] if p["motion_state"] == "MOVING"], "#f0c64e"),
                  ("Approaching", [p for p in r["roi"] if p["direction_state"] == "APPROACHING"], "#f28e2b"))
        for name, points, color in layers:
            if not self.layer_vars[name].get():
                continue
            for p in points:
                x, y = cx + p["x"] * scale, oy - p["y"] * scale
                if 0 < x < w and 0 < y < h:
                    c.create_oval(x-3, y-3, x+3, y+3, fill=color, outline="")
        for obj in r["objects"]:
            if not (self.layer_vars["Clusters"].get() or self.layer_vars["Confirmed"].get() and obj["confirmed"]):
                continue
            x, y = cx + obj["centroid"][0] * scale, oy - obj["centroid"][1] * scale
            color = "#e15759" if obj["target_selected"] else "#67b7dc" if obj["confirmed"] else "#b0aee8"
            radius = 5 if obj["point_count"] == 1 and not obj["confirmed"] else 10
            c.create_oval(x-radius, y-radius, x+radius, y+radius, fill=color, outline="white" if obj["target_selected"] else "")
            c.create_text(x+13, y-10, text=f"OBJ {obj['id']} {obj['distance']:.2f}m", fill="white", anchor="w")

    def _open_object_log(self):
        self._close_object_log()
        path = os.path.join(os.path.dirname(self.cfg_var.get()), datetime.now().strftime("radar_objects_%Y%m%d_%H%M%S.csv"))
        self.log_handle = open(path, "w", newline="", encoding="utf-8-sig")
        self.object_writer = csv.writer(self.log_handle)
        self.object_writer.writerow(OBJECT_FIELDS)
        self.status_var.set(f"Object CSV: {path}")

    def _close_object_log(self):
        if self.log_handle:
            self.log_handle.close()
            self.log_handle = None
            self.object_writer = None

    def _close(self):
        self._stop_live()
        self.destroy()


if __name__ == "__main__":
    RadarApp().mainloop()
