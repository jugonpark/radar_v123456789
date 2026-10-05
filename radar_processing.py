"""Stationary robot radar processing. No motor commands or firmware dependencies."""
from collections import deque
from dataclasses import dataclass
import math
from statistics import median


@dataclass
class Settings:
    min_range_m: float = 0.15
    max_range_m: float = 0.90
    threat_max_range_m: float = 0.90
    min_snr_db: float = 10.0
    require_snr: bool = True
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
    diagnostic_only: bool = False
    target_requires_threat: bool = False
    cluster_before_direction: bool = False
    fast_confidence_on_two_frames: bool = False

    # EXPERIMENTAL: conservative starting values, HARDWARE_TUNING_REQUIRED.
    fast_path_mode: str = "OFF"
    fast_min_track_frames: int = 3
    fast_history_samples: int = 5
    fast_min_range_rate_mps: float = .5
    fast_min_total_closing_m: float = .08
    fast_required_decrease_frames: int = 2
    fast_max_distance_m: float = .9
    fast_min_points_for_strong_evidence: int = 2
    fast_max_angle_jump_deg: float = 15.0

    def validate(self):
        values = vars(self)
        if any(not math.isfinite(float(v)) for k, v in values.items() if k != "fast_path_mode"):
            raise ValueError("Settings must be finite")
        if self.fast_path_mode not in ("OFF", "OBSERVE", "ENABLED"):
            raise ValueError("Invalid fast path mode")
        for name in ("fast_min_track_frames", "fast_history_samples", "fast_required_decrease_frames", "fast_min_points_for_strong_evidence"):
            value = getattr(self, name)
            if int(value) != value or value < 1:
                raise ValueError("Fast path sample counts must be positive integers")
        if self.fast_history_samples < 2 or self.fast_min_track_frames < 2 or self.fast_required_decrease_frames >= self.fast_history_samples:
            raise ValueError("Fast path requires multiple samples and a sufficient history window")
        if any(getattr(self, name) <= 0 for name in ("fast_min_range_rate_mps", "fast_min_total_closing_m", "fast_max_distance_m", "fast_max_angle_jump_deg")):
            raise ValueError("Fast path thresholds must be positive")
        if not 0 <= self.min_range_m < self.max_range_m or self.max_range_m > 100:
            raise ValueError("Invalid range")
        if self.threat_max_range_m <= 0:
            raise ValueError("Invalid threat range")
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
            p["raw_doppler_mps"] = p["doppler"]
            p["motion_state"] = "UNKNOWN"
            p["direction_state"] = "UNKNOWN"
            valid.append(p)
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    return valid


def filter_roi(points, s):
    return [p for p in points if s.min_range_m <= p["distance"] <= s.max_range_m and abs(p["angle"]) <= s.fov_half_angle_deg and (not s.require_snr or p["snr"] is not None and p["snr"] >= s.min_snr_db)]


def classify_motion(points, s):
    for p in points:
        p["motion_state"] = "STATIC" if abs(p["doppler"]) <= s.doppler_deadband_mps else "MOVING"
    return [p for p in points if p["motion_state"] == "MOVING"]


def classify_direction(points, s):
    for p in points:
        speed = s.approach_sign * p["doppler"]
        p["raw_doppler_mps"] = p["doppler"]
        p["approach_doppler_mps"] = speed
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
        velocities = [s.approach_sign * p["doppler"] for p in group]
        approaching_ratio = sum(v > s.doppler_deadband_mps for v in velocities) / len(group)
        representative = median(velocities)
        direction = "APPROACHING" if representative > s.doppler_deadband_mps and approaching_ratio > .5 else "RECEDING" if representative < -s.doppler_deadband_mps else "UNKNOWN"
        result.append(dict(id=None, point_count=len(group), centroid=centroid,
                           distance=median(p["distance"] for p in group),
                           raw_distance=median(p["distance"] for p in group),
                           angle=math.degrees(math.atan2(centroid[0], centroid[1])),
                           raw_doppler_median=median(p["doppler"] for p in group),
                           approach_doppler_median=representative,
                           approach_velocity_from_doppler=representative,
                           doppler_velocity=representative, approaching_point_ratio=approaching_ratio,
                           range_rate_velocity=None, persistence=0, confirmed=False,
                           confidence="LOW", motion_state="MOVING", direction_state=direction,
                           ttc=None, ttc_velocity_source="NONE", risk="SAFE", target_selected=False,
                           pending_reason="WAITING_PERSISTENCE"))
    return result


def estimate_range_rate(history, current_distance, now):
    if not history:
        return None
    previous_time, previous_distance = history[-1]
    dt = now - previous_time
    return (previous_distance - current_distance) / dt if dt > 0 else None


def calculate_ttc(obj, s):
    if not obj["confirmed"] or obj["confidence"] != "HIGH" or obj["direction_state"] != "APPROACHING":
        return None, "NONE"
    speed = obj["range_rate_velocity"]
    if speed is not None and speed > s.doppler_deadband_mps:
        return obj["distance"] / speed, "RANGE_RATE"
    if speed is None and obj["doppler_velocity"] > s.doppler_deadband_mps:
        return obj["distance"] / obj["doppler_velocity"], "DOPPLER"
    return None, "NONE"


def calculate_risk(obj, s):
    ttc = obj["ttc"]
    if ttc is None or obj["distance"] > s.threat_max_range_m:
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
    candidates = [o for o in objects if o["confirmed"] and o["ttc"] is not None and o["confidence"] == "HIGH" and o["direction_state"] == "APPROACHING"]
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
        for point in raw:
            point["approach_doppler_mps"] = s.approach_sign * point["doppler"]
        counts = dict(fast_candidates=0, raw=len(points), valid=len(raw), reject_range=0, reject_fov=0,
                      reject_snr=0, snr_missing=0, snr_min=None, snr_median=None,
                      snr_max=None, roi=0, static=0, moving=0,
                      approaching=0, receding=0, unknown_direction=0,
                      moving_clusters=0, single_point_clusters=0, multi_point_clusters=0,
                      clusters=0, track_matched=0, track_new=0,
                      track_reject_distance=0, track_reject_angle=0, track_reject_speed=0,
                      tentative=0, confirmed=0, low_confidence=0,
                      medium_confidence=0, high_confidence=0, targets=0, threats=0)
        result = dict(frame_id=frame_id, timestamp=now, robot_state=state,
                      processing="ACTIVE" if state == "MONITORING" else "PAUSED",
                      raw=raw, roi=[], clusters=[], objects=[], target=None, legacy_target=None,
                      fast_path=self.fast_output(None), counts=counts)
        if state != "MONITORING":
            self.tracks.clear()
            self.target_id = None
            return result
        if self.last_frame is not None and frame_id <= self.last_frame:
            self.reset()
        self.last_frame = frame_id
        for p in raw:
            if not s.min_range_m <= p["distance"] <= s.max_range_m:
                counts["reject_range"] += 1
            elif abs(p["angle"]) > s.fov_half_angle_deg:
                counts["reject_fov"] += 1
            elif p["snr"] is None:
                counts["snr_missing"] += 1
                if s.require_snr:
                    counts["reject_snr"] += 1
            elif s.require_snr and p["snr"] < s.min_snr_db:
                counts["reject_snr"] += 1
        snr_values = [p["snr"] for p in raw if p["snr"] is not None]
        counts["snr_min"] = min(snr_values) if snr_values else None
        counts["snr_median"] = median(snr_values) if snr_values else None
        counts["snr_max"] = max(snr_values) if snr_values else None
        roi = filter_roi(raw, s)
        moving = classify_motion(roi, s)
        approaching = classify_direction(moving, s)
        clusters = cluster_points(moving if s.cluster_before_direction else approaching, s)
        objects = build_objects(clusters, s)
        counts.update(roi=len(roi), static=len(roi)-len(moving), moving=len(moving),
                      approaching=len(approaching), receding=sum(p["direction_state"] == "RECEDING" for p in moving),
                      unknown_direction=sum(p["direction_state"] == "UNKNOWN" for p in moving),
                      moving_clusters=len(clusters), single_point_clusters=sum(len(c)==1 for c in clusters),
                      multi_point_clusters=sum(len(c)>1 for c in clusters), clusters=len(clusters))
        self.match_objects(objects, now, counts)
        self.update_hysteresis(objects, now)
        for obj in objects:
            obj["ttc"], obj["ttc_velocity_source"] = calculate_ttc(obj, s)
            obj["risk"] = calculate_risk(obj, s)
            obj["pending_reason"] = ("WAITING_PERSISTENCE" if not obj["confirmed"] else
                "NOT_APPROACHING" if obj["direction_state"] != "APPROACHING" else
                "LOW_CONFIDENCE" if obj["confidence"] != "HIGH" else
                "RANGE_TREND_NOT_CONFIRMED" if obj["ttc"] is None else
                "OUTSIDE_THREAT_RANGE" if obj["distance"] > s.threat_max_range_m else
                "NO_IMMEDIATE_THREAT" if obj["risk"] == "SAFE" else "TARGET_READY")
        eligible = objects if not s.target_requires_threat else [o for o in objects if o["risk"] != "SAFE"]
        target = None if s.diagnostic_only else select_target(eligible)
        if target:
            self.target_id = target["id"]
        elif not s.target_requires_threat and self.target_id in self.tracks:
            track = self.tracks[self.target_id]
            if 0 < track["misses"] < s.target_release_misses and track["confirmed"]:
                target = dict(track["last_object"])
                target.update(direction_state="LOST_TEMPORARY", risk="N/A", ttc=None,
                              ttc_velocity_source="NONE", point_count=0, target_selected=True,
                              persistence=sum(track["presence"]))
                target.update(fast_approach_candidate=False, fast_approach_reason="MISSED_DETECTION", misses=track["misses"], consecutive_hits=0,
                              raw_distance_history=[], raw_distance_history_count=0, robust_range_rate=None, instant_range_rate=None,
                              fast_approach_evidence=[], fast_approach_score=0, distance_delta_total=0,
                              consecutive_distance_decreases=0, consecutive_distance_increases=0)
                objects.append(target)
        else:
            self.target_id = None
        legacy_target = dict(target) if target else None
        for obj in objects:
            obj["legacy_target_selected"] = bool(target and obj["id"] == target["id"])
        candidates = [o for o in objects if o.get("fast_approach_candidate") and o["point_count"] > 0]
        fast = min(candidates, key=lambda o: o["raw_distance"] / o["robust_range_rate"], default=None)
        counts["fast_candidates"] = len(candidates)
        if s.fast_path_mode == "ENABLED" and not s.diagnostic_only and fast:
            target = dict(fast)
            target.update(distance=fast["raw_distance"], range_rate_velocity=fast["robust_range_rate"],
                          confidence="HIGH", confirmed=True, direction_state="APPROACHING",
                          ttc=fast["raw_distance"] / fast["robust_range_rate"],
                          ttc_velocity_source="EXPERIMENTAL_RAW_RANGE_RATE", target_selected=True)
            target["risk"] = calculate_risk(target, s)
            for obj in objects:
                obj["target_selected"] = obj["id"] == target["id"]
        counts["tentative"] = sum(not o["confirmed"] for o in objects)
        counts["confirmed"] = sum(o["confirmed"] for o in objects)
        for confidence in ("LOW", "MEDIUM", "HIGH"):
            counts[confidence.lower() + "_confidence"] = sum(o["confidence"] == confidence for o in objects)
        counts["targets"] = int(target is not None)
        counts["threats"] = sum(o["risk"] not in ("SAFE", "N/A") for o in objects)
        result.update(roi=roi, clusters=clusters, objects=objects, target=target, legacy_target=legacy_target, fast_path=self.fast_output(fast))
        return result

    def match_objects(self, objects, now, counts=None):
        s = self.settings
        if counts is not None:
            for o in objects:
                for t in self.tracks.values():
                    if not t["history"] or not 0 < now - t["history"][-1][0] <= .5:
                        continue
                    if abs(o["angle"] - t["angle"]) > s.track_match_angle_deg:
                        counts["track_reject_angle"] += 1
                    elif abs(o["distance"] - t["history"][-1][1]) > s.max_track_speed_mps * (now - t["history"][-1][0]) + .03:
                        counts["track_reject_speed"] += 1
                    elif math.dist(o["centroid"], t["centroid"]) > s.track_match_distance_m:
                        counts["track_reject_distance"] += 1
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
                if counts is not None: counts["track_matched"] += 1
                used_objects.add(oi)
                used_tracks.add(tid)
        for obj in objects:
            if obj["id"] is None:
                obj["id"] = self.next_id
                if counts is not None: counts["track_new"] += 1
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
            self.update_fast_diagnostics(track, obj, now)
            track["presence"] = deque(track["presence"], maxlen=s.temporal_window)
            if track["history"]:
                previous_distance = track["history"][-1][1]
                obj["distance"] = previous_distance + s.distance_smoothing_alpha * (obj["raw_distance"] - previous_distance)
            instantaneous = estimate_range_rate(track["history"], obj["distance"], now)
            if instantaneous is not None:
                track["rates"].append(instantaneous)
            obj["range_rate_velocity"] = median(track["rates"]) if track["rates"] else None
            track["history"].append((now, obj["distance"]))
            obj["smoothed_distance"] = obj["distance"]
            obj["distance_history"] = list(track["history"])
            obj["distance_history_count"] = len(track["history"])
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
            fast_evidence = (s.fast_confidence_on_two_frames and len(track["rates"]) >= 1
                             and trend is not None and trend > s.doppler_deadband_mps
                             and obj["doppler_velocity"] > s.doppler_deadband_mps
                             and obj["approaching_point_ratio"] > .5)
            obj["confidence"] = "HIGH" if obj["confirmed"] and obj["direction_state"] == "APPROACHING" and (consistent or fast_evidence) else "MEDIUM" if obj["confirmed"] and (trend is None or trend > s.doppler_deadband_mps) else "LOW"
            track["last_object"] = dict(obj)
        for tid, track in list(self.tracks.items()):
            if tid in seen:
                continue
            track["presence"].append(False)
            track["misses"] += 1
            track["consecutive_hits"] = 0
            track["raw_history"] = deque(maxlen=s.fast_history_samples)
            track["angle_history"] = deque(maxlen=s.fast_history_samples)
            if track["misses"] >= s.target_release_misses:
                del self.tracks[tid]

    def fast_output(self, obj):
        return dict(mode=self.settings.fast_path_mode, candidate=obj is not None,
                    track_id=obj["id"] if obj else None,
                    distance=obj["raw_distance"] if obj else None,
                    range_rate=obj["robust_range_rate"] if obj else None,
                    doppler_support="DOPPLER_SUPPORT" in obj["fast_approach_evidence"] if obj else False,
                    evidence=obj["fast_approach_evidence"] if obj else [],
                    reason=obj["fast_approach_reason"] if obj else ("OFF" if self.settings.fast_path_mode == "OFF" else "NO_CANDIDATE"))

    def update_fast_diagnostics(self, track, obj, now):
        s = self.settings
        history = deque(track.get("raw_history", ()), maxlen=s.fast_history_samples)
        angles = deque(track.get("angle_history", ()), maxlen=s.fast_history_samples)
        # Existing matching timeout is .5 s; never bridge frame gaps or misses.
        continuous = (history and 0 < now-history[-1][0] <= .5 and
                      track.get("fast_frame") == self.last_frame-1 and not track["misses"])
        if not continuous:
            history.clear(); angles.clear()
            track["consecutive_hits"] = 0
        previous = history[-1] if history else None
        angle_delta = obj["angle"]-angles[-1] if angles else 0.0
        history.append((now, obj["raw_distance"]))
        angles.append(obj["angle"])
        track["raw_history"], track["angle_history"] = history, angles
        track["fast_frame"] = self.last_frame
        track.setdefault("first_frame", self.last_frame)
        track["age_frames"] = self.last_frame-track["first_frame"]+1
        track["consecutive_hits"] = track.get("consecutive_hits", 0)+1
        rates = [(a[1]-b[1])/(b[0]-a[0]) for i,a in enumerate(history)
                 for b in list(history)[i+1:] if b[0]>a[0]]
        decrease = increase = 0
        for a,b in reversed(list(zip(history, list(history)[1:]))):
            if b[1] < a[1]-1e-6: decrease += 1
            else: break
        for a,b in reversed(list(zip(history, list(history)[1:]))):
            if b[1] > a[1]+1e-6: increase += 1
            else: break
        delta = history[-1][1]-history[0][1]
        robust = median(rates) if rates else None
        evidence = []
        mandatory = {
            "TRACK_PERSISTENCE": track["consecutive_hits"] >= s.fast_min_track_frames,
            "RANGE_CLOSING": decrease >= s.fast_required_decrease_frames and -delta >= s.fast_min_total_closing_m,
            "FAST_RANGE_RATE": robust is not None and robust >= s.fast_min_range_rate_mps,
            "DISTANCE_RELEVANT": obj["raw_distance"] <= s.fast_max_distance_m,
            "TRACK_STABLE": max(angles)-min(angles) <= s.fast_max_angle_jump_deg,
        }
        evidence.extend(k for k,v in mandatory.items() if v)
        if obj["doppler_velocity"] > s.doppler_deadband_mps: evidence.append("DOPPLER_SUPPORT")
        if obj["point_count"] >= s.fast_min_points_for_strong_evidence: evidence.append("MULTI_POINT_SUPPORT")
        candidate = s.fast_path_mode != "OFF" and all(mandatory.values())
        reason = "OFF" if s.fast_path_mode == "OFF" else "FAST_APPROACH_EVIDENCE" if candidate else "MISSING:"+",".join(k for k,v in mandatory.items() if not v)
        obj.update(track_id=obj["id"], track_age_frames=track["age_frames"],
                   consecutive_hits=track["consecutive_hits"], misses=0,
                   centroid_x=obj["centroid"][0], centroid_y=obj["centroid"][1], centroid_z=obj["centroid"][2],
                   raw_distance_history=list(history), raw_distance_history_count=len(history),
                   instant_range_rate=(previous[1]-obj["raw_distance"])/(now-previous[0]) if previous else None,
                   robust_range_rate=robust, distance_delta_total=delta,
                   consecutive_distance_decreases=decrease, consecutive_distance_increases=increase,
                   angle_delta=angle_delta, angle_stability=max(angles)-min(angles),
                   fast_approach_candidate=candidate, fast_approach_evidence=evidence,
                   fast_approach_score=len(evidence), fast_approach_reason=reason,
                   legacy_target_selected=False)
