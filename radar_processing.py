"""Stationary robot radar processing. No motor commands or firmware dependencies."""
from collections import deque
from dataclasses import dataclass
import math
from statistics import median


@dataclass
class Settings:
    min_range_m: float = 0.15
    max_range_m: float = 0.90
    min_snr_db: float = 10.0
    fov_half_angle_deg: float = 60.0
    doppler_deadband_mps: float = 0.10
    approach_sign: int = -1
    cluster_distance_m: float = 0.20
    min_cluster_points: int = 2
    track_match_distance_m: float = 0.30
    track_match_angle_deg: float = 20.0
    max_track_speed_mps: float = 2.0
    distance_smoothing_alpha: float = 0.35
    temporal_window: int = 3
    temporal_required: int = 3
    target_release_misses: int = 3
    settling_time_s: float = 0.30
    watch_ttc_s: float = 3.0
    warning_ttc_s: float = 2.0
    avoid_ttc_s: float = 1.2
    stop_ttc_s: float = 0.6
    emergency_distance_m: float = 0.20

    def validate(self):
        values = vars(self)
        if any(not math.isfinite(float(v)) for v in values.values()):
            raise ValueError("Settings must be finite")
        if not 0 <= self.min_range_m < self.max_range_m or self.max_range_m > 100:
            raise ValueError("Invalid range")
        if not 0 < self.fov_half_angle_deg <= 180 or self.approach_sign not in (-1, 1):
            raise ValueError("Invalid FOV or approach sign")
        if any(getattr(self, x) <= 0 for x in ("cluster_distance_m", "track_match_distance_m", "track_match_angle_deg", "max_track_speed_mps", "temporal_window", "temporal_required", "target_release_misses", "min_cluster_points")):
            raise ValueError("Cluster and temporal settings must be positive")
        if self.temporal_required > self.temporal_window:
            raise ValueError("Required detections exceed temporal window")
        if not 0 < self.distance_smoothing_alpha <= 1:
            raise ValueError("Distance smoothing alpha must be in (0, 1]")
        if not 0 < self.stop_ttc_s <= self.avoid_ttc_s <= self.warning_ttc_s <= self.watch_ttc_s:
            raise ValueError("TTC thresholds must be ascending")
        if self.settling_time_s < 0 or self.doppler_deadband_mps < 0:
            raise ValueError("Invalid settling time or Doppler deadband")


class RobotStateManager:
    def __init__(self, settling_time_s=0.30):
        self.state = "MONITORING"
        self.transition_time = 0.0
        self.settling_time_s = settling_time_s

    def set_moving(self, moving, now):
        if moving:
            self.state = "MOVING"
            self.transition_time = now
        elif self.state == "MOVING":
            self.state = "SETTLING"
            self.transition_time = now

    def update(self, now):
        if self.state == "SETTLING" and now - self.transition_time >= self.settling_time_s:
            self.state = "MONITORING"
        return self.state


def validate_points(points):
    valid = []
    for raw in points:
        try:
            p = {k: float(raw[k]) for k in ("x", "y", "z", "doppler")}
            p["snr"] = None if raw.get("snr") in (None, "") else float(raw["snr"])
            p["noise"] = None if raw.get("noise") in (None, "") else float(raw["noise"])
            if not all(math.isfinite(v) for v in p.values() if v is not None):
                continue
            if max(abs(p[k]) for k in ("x", "y", "z")) > 100:
                continue
            p["distance"] = math.sqrt(sum(p[k] ** 2 for k in ("x", "y", "z")))
            if p["distance"] <= 0:
                continue
            p["angle"] = math.degrees(math.atan2(p["x"], p["y"]))
            p["motion_state"] = "UNKNOWN"
            p["direction_state"] = "UNKNOWN"
            valid.append(p)
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    return valid


def filter_roi(points, s):
    return [p for p in points if s.min_range_m <= p["distance"] <= s.max_range_m and abs(p["angle"]) <= s.fov_half_angle_deg and p["snr"] is not None and p["snr"] >= s.min_snr_db]


def classify_motion(points, s):
    for p in points:
        p["motion_state"] = "STATIC" if abs(p["doppler"]) <= s.doppler_deadband_mps else "MOVING"
    return [p for p in points if p["motion_state"] == "MOVING"]


def classify_direction(points, s):
    for p in points:
        speed = s.approach_sign * p["doppler"]
        p["direction_state"] = "APPROACHING" if speed > s.doppler_deadband_mps else "RECEDING" if speed < -s.doppler_deadband_mps else "UNKNOWN"
    return [p for p in points if p["direction_state"] == "APPROACHING"]


def cluster_points(points, s):
    remaining = set(range(len(points)))
    clusters = []
    while remaining:
        component = {remaining.pop()}
        frontier = list(component)
        while frontier:
            i = frontier.pop()
            neighbors = {j for j in remaining if math.hypot(points[i]["x"] - points[j]["x"], points[i]["y"] - points[j]["y"]) <= s.cluster_distance_m}
            remaining -= neighbors
            component |= neighbors
            frontier.extend(neighbors)
        if len(component) >= s.min_cluster_points:
            clusters.append([points[i] for i in sorted(component)])
    return clusters


def build_objects(clusters, s):
    result = []
    for group in clusters:
        centroid = tuple(median(p[k] for p in group) for k in ("x", "y", "z"))
        result.append(dict(id=None, point_count=len(group), centroid=centroid,
                           distance=median(p["distance"] for p in group),
                           raw_distance=median(p["distance"] for p in group),
                           angle=math.degrees(math.atan2(centroid[0], centroid[1])),
                           doppler_velocity=median(s.approach_sign * p["doppler"] for p in group),
                           range_rate_velocity=None, persistence=0, confirmed=False,
                           confidence="LOW", motion_state="MOVING", direction_state="APPROACHING",
                           ttc=None, ttc_velocity_source="NONE", risk="SAFE", target_selected=False))
    return result


def estimate_range_rate(history, current_distance, now):
    if not history:
        return None
    previous_time, previous_distance = history[-1]
    dt = now - previous_time
    return (previous_distance - current_distance) / dt if dt > 0 else None


def calculate_ttc(obj, s):
    if not obj["confirmed"] or obj["confidence"] != "HIGH":
        return None, "NONE"
    speed = obj["range_rate_velocity"]
    if speed is not None and speed > s.doppler_deadband_mps:
        return obj["distance"] / speed, "RANGE_RATE"
    if speed is None and obj["doppler_velocity"] > s.doppler_deadband_mps:
        return obj["distance"] / obj["doppler_velocity"], "DOPPLER"
    return None, "NONE"


def calculate_risk(obj, s):
    ttc = obj["ttc"]
    if ttc is None:
        return "SAFE"
    if obj["distance"] <= s.emergency_distance_m or ttc <= s.stop_ttc_s:
        return "STOP"
    if ttc <= s.avoid_ttc_s:
        return "AVOID"
    if ttc <= s.warning_ttc_s:
        return "WARNING"
    if ttc <= s.watch_ttc_s:
        return "WATCH"
    return "SAFE"


def select_target(objects):
    candidates = [o for o in objects if o["confirmed"] and o["ttc"] is not None and o["confidence"] == "HIGH"]
    target = min(candidates, key=lambda o: o["ttc"], default=None)
    if target:
        target["target_selected"] = True
    return target


class RadarProcessor:
    def __init__(self, settings=None):
        self.settings = settings or Settings()
        self.robot = RobotStateManager(self.settings.settling_time_s)
        self.tracks = {}
        self.next_id = 1
        self.last_frame = None
        self.target_id = None

    def reset(self):
        self.tracks.clear()
        self.next_id = 1
        self.last_frame = None
        self.target_id = None

    def set_moving(self, moving, now):
        previous = self.robot.state
        self.robot.set_moving(moving, now)
        if previous != self.robot.state:
            self.tracks.clear()
            self.target_id = None

    def process_frame(self, frame_id, points, now):
        s = self.settings
        s.validate()
        self.robot.settling_time_s = s.settling_time_s
        state = self.robot.update(now)
        raw = validate_points(points)
        counts = dict(raw=len(points), valid=len(raw), roi=0, static=0, moving=0,
                      approaching=0, receding=0, clusters=0, confirmed=0, threats=0)
        result = dict(frame_id=frame_id, timestamp=now, robot_state=state,
                      processing="ACTIVE" if state == "MONITORING" else "PAUSED",
                      raw=raw, roi=[], clusters=[], objects=[], target=None, counts=counts)
        if state != "MONITORING":
            self.tracks.clear()
            self.target_id = None
            return result
        if self.last_frame is not None and frame_id <= self.last_frame:
            self.reset()
        self.last_frame = frame_id
        roi = filter_roi(raw, s)
        moving = classify_motion(roi, s)
        approaching = classify_direction(moving, s)
        clusters = cluster_points(approaching, s)
        objects = build_objects(clusters, s)
        counts.update(roi=len(roi), static=len(roi)-len(moving), moving=len(moving),
                      approaching=len(approaching), receding=sum(p["direction_state"] == "RECEDING" for p in moving), clusters=len(clusters))
        self.match_objects(objects, now)
        self.update_hysteresis(objects, now)
        for obj in objects:
            obj["ttc"], obj["ttc_velocity_source"] = calculate_ttc(obj, s)
            obj["risk"] = calculate_risk(obj, s)
        target = select_target(objects)
        if target:
            self.target_id = target["id"]
        elif self.target_id in self.tracks:
            track = self.tracks[self.target_id]
            if 0 < track["misses"] < s.target_release_misses and track["confirmed"]:
                target = dict(track["last_object"])
                target.update(direction_state="LOST_TEMPORARY", risk="N/A", ttc=None,
                              ttc_velocity_source="NONE", point_count=0, target_selected=True,
                              persistence=sum(track["presence"]))
                objects.append(target)
        else:
            self.target_id = None
        counts["confirmed"] = sum(o["confirmed"] for o in objects)
        counts["threats"] = sum(o["risk"] not in ("SAFE", "N/A") for o in objects)
        result.update(roi=roi, clusters=clusters, objects=objects, target=target)
        return result

    def match_objects(self, objects, now):
        s = self.settings
        pairs = sorted((math.dist(o["centroid"], t["centroid"]), oi, tid)
                       for oi, o in enumerate(objects) for tid, t in self.tracks.items()
                       if t["history"] and 0 < now - t["history"][-1][0] <= 0.5
                       and abs(o["angle"] - t["angle"]) <= s.track_match_angle_deg
                       and abs(o["distance"] - t["history"][-1][1]) <=
                       s.max_track_speed_mps * (now - t["history"][-1][0]) + 0.03)
        used_objects, used_tracks = set(), set()
        for distance, oi, tid in pairs:
            if distance <= s.track_match_distance_m and oi not in used_objects and tid not in used_tracks:
                objects[oi]["id"] = tid
                used_objects.add(oi)
                used_tracks.add(tid)
        for obj in objects:
            if obj["id"] is None:
                obj["id"] = self.next_id
                self.next_id += 1
                self.tracks[obj["id"]] = dict(centroid=obj["centroid"], angle=obj["angle"], history=deque(maxlen=8), rates=deque(maxlen=3),
                                               presence=deque(maxlen=s.temporal_window), misses=0, confirmed=False)

    def update_hysteresis(self, objects, now):
        s = self.settings
        seen = set()
        for obj in objects:
            tid = obj["id"]
            seen.add(tid)
            track = self.tracks[tid]
            track["presence"] = deque(track["presence"], maxlen=s.temporal_window)
            if track["history"]:
                previous_distance = track["history"][-1][1]
                obj["distance"] = previous_distance + s.distance_smoothing_alpha * (obj["raw_distance"] - previous_distance)
            instantaneous = estimate_range_rate(track["history"], obj["distance"], now)
            if instantaneous is not None:
                track["rates"].append(instantaneous)
            obj["range_rate_velocity"] = median(track["rates"]) if track["rates"] else None
            track["history"].append((now, obj["distance"]))
            track["centroid"] = obj["centroid"]
            track["angle"] = obj["angle"]
            track["presence"].append(True)
            track["misses"] = 0
            if sum(track["presence"]) >= s.temporal_required:
                track["confirmed"] = True
            obj["confirmed"] = track["confirmed"]
            obj["persistence"] = sum(track["presence"])
            trend = obj["range_rate_velocity"]
            consistent = len(track["rates"]) >= 2 and all(rate > s.doppler_deadband_mps for rate in list(track["rates"])[-2:])
            obj["confidence"] = "HIGH" if obj["confirmed"] and consistent else "MEDIUM" if obj["confirmed"] and (trend is None or trend > s.doppler_deadband_mps) else "LOW"
            track["last_object"] = dict(obj)
        for tid, track in list(self.tracks.items()):
            if tid in seen:
                continue
            track["presence"].append(False)
            track["misses"] += 1
            if track["misses"] >= s.target_release_misses:
                del self.tracks[tid]
