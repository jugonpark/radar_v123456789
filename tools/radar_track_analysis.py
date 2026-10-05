"""Track observations, never object identity or labelled hardware ground truth."""
import math
import statistics
from collections import defaultdict


def median(values):
    values = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    return statistics.median(values) if values else None


class TrackAnalysis:
    def __init__(self, action_start=None):
        self.action_start = action_start
        self.rows = defaultdict(list)
        self.frames = 0
        self.receives = []
        self.fast_frames = self.high_frames = self.target_frames = 0
        self.fast_run = self.max_fast_run = 0
        self.new = self.matched = 0
        self.approach_frames = self.approach_fast_frames = self.avoid_frames = self.receding_frames = 0
        self.approach_speeds, self.primary_distances, self.primary_ids = [], [], []
        self.first_approach_fast = self.first_avoid = None
        self.approach_false_positives = []

    def add(self, frame, timestamp, result):
        self.frames += 1
        self.receives.append((frame, timestamp))
        approach = result.get("approach_decision", {})
        self.approach_frames += approach.get("decision") in ("WATCH", "AVOID_CANDIDATE")
        fast_approach = any(o.get("state") == "FAST_APPROACH" for o in approach.get("objects", []))
        self.approach_fast_frames += fast_approach
        self.receding_frames += any(o.get("state") == "RECEDING" for o in approach.get("objects", []))
        avoid = bool(approach.get("avoid_candidate"))
        self.avoid_frames += avoid
        if fast_approach and self.first_approach_fast is None: self.first_approach_fast = timestamp
        if avoid and self.first_avoid is None: self.first_avoid = timestamp
        primary = approach.get("primary_object")
        if primary and approach.get("current_frame_valid"):
            self.primary_ids.append(primary["track_id"])
            if primary.get("approach_speed_mps") is not None: self.approach_speeds.append(primary["approach_speed_mps"])
            if primary.get("distance_m") is not None: self.primary_distances.append(primary["distance_m"])
        for obj in approach.get("objects", []):
            if obj.get("avoid_candidate") or obj.get("state") == "FAST_APPROACH":
                self.approach_false_positives.append(dict(frame=frame, timestamp=timestamp, **obj))
        objects = [o for o in result.get("objects", []) if o.get("misses", 0) == 0 and o.get("point_count", 1) > 0]
        fast = any(o.get("fast_approach_candidate", False) for o in objects)
        self.fast_frames += fast
        self.high_frames += any(o.get("confidence") == "HIGH" for o in objects)
        legacy = result.get("legacy_target", result.get("target"))
        self.target_frames += legacy is not None
        self.fast_run = self.fast_run + 1 if fast else 0
        self.max_fast_run = max(self.max_fast_run, self.fast_run)
        self.new += result.get("counts", {}).get("track_new", 0)
        self.matched += result.get("counts", {}).get("track_matched", 0)
        for obj in objects:
            row = dict(obj, frame=frame, timestamp=timestamp, receive_index=self.frames)
            row["legacy_target_selected"] = bool(legacy and legacy.get("id") == obj.get("id"))
            self.rows[obj.get("track_id", obj["id"])].append(row)

    def summary(self):
        tracks = []
        intervals = sorted(b[1]-a[1] for a,b in zip(self.receives,self.receives[1:]) if b[1] > a[1])
        cadence = statistics.median(intervals[:max(1, len(intervals)//2)]) if intervals else None
        def continuous_lifetime(rows):
            longest = run = 1
            for a, b in zip(rows, rows[1:]):
                delta = b["timestamp"]-a["timestamp"]
                adjacent = (b["receive_index"] == a["receive_index"]+1 and
                            ((b["frame"]-a["frame"]) & 0xffffffff) == 1 and delta > 0 and
                            (cadence is None or delta <= cadence*3))
                run = run+1 if adjacent else 1
                longest = max(longest,run)
            return longest

        for ident, rows in self.rows.items():
            distances = [r.get("raw_distance", r.get("distance")) for r in rows]
            distances = [v for v in distances if v is not None]
            rates = [r.get("robust_range_rate") for r in rows if r.get("robust_range_rate") is not None]
            angles = [r.get("angle", 0) for r in rows]
            fast = [r for r in rows if r.get("fast_approach_candidate")]
            tracks.append(dict(track_id=ident, first_frame=rows[0]["frame"], last_frame=rows[-1]["frame"],
                lifetime_frames=len(rows), max_continuous_lifetime=continuous_lifetime(rows), first_distance=distances[0] if distances else None,
                minimum_distance=min(distances) if distances else None, maximum_distance=max(distances) if distances else None,
                last_distance=distances[-1] if distances else None,
                total_distance_change=distances[-1]-distances[0] if distances else 0,
                robust_range_rate_median=median(rates), robust_range_rate_max=max(rates) if rates else None,
                consecutive_decrease_max=max(r.get("consecutive_distance_decreases", 0) for r in rows),
                consecutive_increase_max=max(r.get("consecutive_distance_increases", 0) for r in rows),
                raw_doppler_median=median([r.get("raw_doppler_median") for r in rows]),
                approach_doppler_median=median([r.get("approach_doppler_median") for r in rows]),
                median_point_count=median([r.get("point_count", 0) for r in rows]),
                mean_angle=statistics.mean(angles), angle_span=max(angles)-min(angles),
                confirmed_frame_count=sum(bool(r.get("confirmed")) for r in rows),
                high_confidence_frame_count=sum(r.get("confidence") == "HIGH" for r in rows),
                target_frame_count=sum(r["legacy_target_selected"] for r in rows), fast_candidate_frame_count=len(fast),
                fast_candidate_first_detection_frame=fast[0]["frame"] if fast else None,
                fast_candidate_first_detection_time=fast[0]["timestamp"] if fast else None,
                first_detection_latency_from_action_start=(fast[0]["timestamp"]-self.action_start) if fast and self.action_start is not None else None))
        first = min((t["fast_candidate_first_detection_time"] for t in tracks if t["fast_candidate_first_detection_time"] is not None), default=None)
        short = sum(t["lifetime_frames"] <= 2 for t in tracks)
        return dict(frames=self.frames, approach_decision_frame_fraction=self.approach_frames/self.frames if self.frames else 0,
            fast_approach_frame_fraction=self.approach_fast_frames/self.frames if self.frames else 0,
            avoid_candidate_frame_fraction=self.avoid_frames/self.frames if self.frames else 0,
            receding_frame_fraction=self.receding_frames/self.frames if self.frames else 0,
            max_approach_speed_mps=max(self.approach_speeds, default=None), median_approach_speed_mps=median(self.approach_speeds),
            minimum_primary_distance_m=min(self.primary_distances, default=None),
            primary_track_id=min(set(self.primary_ids),key=lambda i:(-self.primary_ids.count(i),i)) if self.primary_ids else None,
            primary_track_ids=sorted(set(self.primary_ids)),
            first_fast_approach_latency=self.first_approach_fast-self.action_start if self.first_approach_fast is not None and self.action_start is not None else None,
            first_avoid_candidate_latency=self.first_avoid-self.action_start if self.first_avoid is not None and self.action_start is not None else None,
            approach_metric_scope="WATCH/AVOID frame fraction; signed speeds and distances from usable primary objects; latency from scenario START, not motion onset",
            tracks=tracks, longest_track=max(tracks, key=lambda t:t["lifetime_frames"], default=None),
            largest_closing_track=min(tracks, key=lambda t:t["total_distance_change"], default=None),
            fastest_closing_track=max(tracks, key=lambda t:t["robust_range_rate_max"] or 0, default=None),
            legacy_target_fraction=self.target_frames/self.frames if self.frames else 0,
            legacy_high_fraction=self.high_frames/self.frames if self.frames else 0,
            fast_candidate_frame_fraction=self.fast_frames/self.frames if self.frames else 0,
            fast_candidate_fraction=self.fast_frames/self.frames if self.frames else 0,
            fast_candidate_first_detection=first,
            first_detection_latency_from_action_start=first-self.action_start if first is not None and self.action_start is not None else None,
            max_consecutive_fast_frames=self.max_fast_run, track_new=self.new, matched=self.matched,
            short_track_count=short, max_continuous_lifetime=max((t["max_continuous_lifetime"] for t in tracks), default=0),
            track_fragmentation_estimate=short/len(tracks) if tracks else None,
            fragmentation_note="Short-track fraction is an estimate, not hand identity ground truth.",
            diagnostics=["TRACK_MATCHING_MAY_BE_TOO_STRICT"] if self.new > self.matched and short > 2 else [])


TRACK_COLUMNS = "scenario phase frame timestamp track_id track_age_frames consecutive_hits misses centroid_x centroid_y centroid_z distance_history_count raw_distance_history_count point_count raw_distance smoothed_distance robust_range_rate instant_range_rate distance_delta_total consecutive_distance_decreases consecutive_distance_increases raw_doppler_median approach_doppler_median approaching_point_ratio angle angle_stability confirmed confidence legacy_target_selected fast_approach_candidate fast_approach_score fast_approach_reason approach_state approach_avoid_candidate approach_speed_mps approach_ttc_s approach_confidence approach_reason".split()


def track_csv_row(scenario, phase, frame, timestamp, obj, legacy):
    row = dict(obj, scenario=scenario, phase=phase, frame=frame, timestamp=timestamp,
               track_id=obj.get("track_id", obj.get("id")), legacy_target_selected=bool(legacy and legacy.get("id") == obj.get("id")))
    approach = obj.get("approach_decision", {})
    row.update(approach_state=approach.get("state"), approach_avoid_candidate=approach.get("avoid_candidate"),
               approach_speed_mps=approach.get("approach_speed_mps"), approach_ttc_s=approach.get("ttc_s"),
               approach_confidence=approach.get("confidence"), approach_reason=";".join(approach.get("reason", [])))
    row["distance_history_count"] = len(obj.get("distance_history", []))
    row["raw_distance_history_count"] = len(obj.get("raw_distance_history", []))
    return {k: row.get(k) for k in TRACK_COLUMNS}


def false_positive_diagnostics(name, analysis):
    if name not in ("SCENE_0_EMPTY", "SCENE_1_STATIC_HAND", "SCENE_4_RECEDE", "SCENE_5_SIDE_MOTION"):
        return []
    return [dict(label="FAST_FALSE_POSITIVE", interpretation="Scenario-inconsistent evidence, not verified object attribution", track=r.get("track_id", r.get("id")),
        reason=r.get("fast_approach_reason"), range_rate=r.get("robust_range_rate"), doppler=r.get("approach_doppler_median"),
        distance_delta=r.get("distance_delta_total"), age=r.get("track_age_frames"), points=r.get("point_count"), angle_span=r.get("angle_stability"))
        for rows in analysis.rows.values() for r in rows if r.get("fast_approach_candidate")]


def scenario_assessment(name, summary):
    """Label-consistent observed evidence, not accuracy certification."""
    if not summary.get("frames", 0):
        return dict(status="UNKNOWN", frames=0, reason="No observed action-window frames")
    tracks = summary["tracks"]
    closing = [t for t in tracks if t["lifetime_frames"] >= 2 and t["total_distance_change"] < -.02]
    increasing = [t for t in tracks if t["lifetime_frames"] >= 2 and t["total_distance_change"] > .02]
    if name == "SCENE_2_SLOW_APPROACH":
        observed = any((t["robust_range_rate_max"] or 0) > 0 for t in closing)
    elif name == "SCENE_3_FAST_APPROACH":
        observed = summary["fast_candidate_fraction"] > 0
    elif name == "SCENE_4_RECEDE":
        observed = bool(increasing) and summary["fast_candidate_fraction"] == 0
    else:
        observed = summary["fast_candidate_fraction"] == 0 and summary["legacy_target_fraction"] == 0 and not closing
    return dict(status="PASS" if observed else "WARN", frames=summary["frames"], closing_track_count=len(closing), increasing_track_count=len(increasing),
                fast_candidate_fraction=summary["fast_candidate_fraction"], legacy_target_fraction=summary["legacy_target_fraction"],
                scope="Scenario-labelled track evidence only; repeat trials and verify attribution")


APPROACH_SCENE_NAMES = ("SCENE_0_EMPTY", "SCENE_3_FAST_APPROACH", "SCENE_4_RECEDE", "SCENE_5_SIDE_MOTION")
APPROACH_OPTIONS = {"approach-fast-speed":("fast_approach_speed_mps", float),
    "approach-avoid-distance":("approach_avoid_max_distance_m", float),
    "approach-observe-distance":("approach_observe_max_distance_m", float),
    "approach-min-track-frames":("approach_min_track_frames", int),
    "approach-speed-deadband":("approach_speed_deadband_mps", float)}


def add_approach_arguments(parser):
    parser.add_argument("--approach-path-mode", choices=("OFF", "OBSERVE"))
    for flag, (_,kind) in APPROACH_OPTIONS.items(): parser.add_argument("--"+flag,type=kind)


def apply_approach_arguments(settings, args):
    for flag,(field,_) in APPROACH_OPTIONS.items():
        value=getattr(args,flag.replace("-","_"))
        if value is not None:setattr(settings,field,value)
    if args.approach_path_mode is not None:settings.approach_path_mode=args.approach_path_mode
    settings.validate()


def approach_false_positive_diagnostics(name, analysis):
    if name not in ("SCENE_0_EMPTY", "SCENE_1_STATIC_HAND", "SCENE_4_RECEDE", "SCENE_5_SIDE_MOTION"):
        return []
    return [dict(label="APPROACH_SCENARIO_INCONSISTENT", scenario=name, interpretation="Observed evidence, not identified hand ground truth", **row)
            for row in analysis.approach_false_positives]


def approach_check(scenarios, failure=None, required=APPROACH_SCENE_NAMES):
    indexed={s["name"]:s for s in scenarios}
    missing=[n for n in required if n not in indexed]
    empty=[s["name"] for s in scenarios if not s["action_window"].get("frames",0)]
    incomplete=[s["name"] for s in scenarios if not s.get("capture_complete",False)]
    complete=not missing and not empty and not incomplete and not failure
    positive=any(s["name"] == "SCENE_3_FAST_APPROACH" and s["action_window"].get("avoid_candidate_frame_fraction",0)>0 for s in scenarios)
    retreat=any(s["name"] == "SCENE_4_RECEDE" and s["action_window"].get("receding_frame_fraction",0)>0 for s in scenarios)
    contradictory=any(s.get("approach_false_positive_diagnostics") for s in scenarios)
    return dict(status="UNKNOWN" if not complete else "PASS" if positive and retreat and not contradictory else "WARN",
        reason="Complete scenario evidence; repeat hardware trials and verify object attribution", coverage=len(indexed),
        required_scenes=list(required), missing_scenes=missing, empty_action_windows=empty, incomplete_captures=incomplete,
        failure=failure, positive_action_avoid=positive, receding_action_observed=retreat, inconsistent_observations=contradictory,
        metrics={s["name"]:{k:v for k,v in s["action_window"].items() if k in ("frames","fast_approach_frame_fraction","avoid_candidate_frame_fraction","max_approach_speed_mps","first_avoid_candidate_latency")} for s in scenarios})


def approach_console(decision):
    primary=decision.get("primary_object")
    if not primary:return "[APPROACH] track=N/A d=N/A v=N/A state=UNKNOWN avoid=0"
    speed=primary.get("approach_speed_mps")
    distance=primary.get("distance_m")
    velocity=f"{speed:+.2f}m/s" if speed is not None else "N/A"
    range_text=f"{distance:.2f}m" if distance is not None else "N/A"
    return f"[APPROACH] track={primary['track_id']} d={range_text} v={velocity} state={primary['state']} avoid={int(primary['avoid_candidate'])}"
