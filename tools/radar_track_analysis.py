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
        self.fast_frames = self.high_frames = self.target_frames = 0
        self.fast_run = self.max_fast_run = 0
        self.new = self.matched = 0

    def add(self, frame, timestamp, result):
        self.frames += 1
        objects = result.get("objects", [])
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
            row = dict(obj, frame=frame, timestamp=timestamp)
            row["legacy_target_selected"] = bool(legacy and legacy.get("id") == obj.get("id"))
            self.rows[obj.get("track_id", obj["id"])].append(row)

    def summary(self):
        tracks = []
        for ident, rows in self.rows.items():
            distances = [r.get("raw_distance", r.get("distance")) for r in rows]
            distances = [v for v in distances if v is not None]
            rates = [r.get("robust_range_rate") for r in rows if r.get("robust_range_rate") is not None]
            angles = [r.get("angle", 0) for r in rows]
            fast = [r for r in rows if r.get("fast_approach_candidate")]
            tracks.append(dict(track_id=ident, first_frame=rows[0]["frame"], last_frame=rows[-1]["frame"],
                lifetime_frames=len(rows), first_distance=distances[0] if distances else None,
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
        return dict(tracks=tracks, longest_track=max(tracks, key=lambda t:t["lifetime_frames"], default=None),
            largest_closing_track=min(tracks, key=lambda t:t["total_distance_change"], default=None),
            fastest_closing_track=max(tracks, key=lambda t:t["robust_range_rate_max"] or 0, default=None),
            legacy_target_fraction=self.target_frames/self.frames if self.frames else 0,
            legacy_high_fraction=self.high_frames/self.frames if self.frames else 0,
            fast_candidate_frame_fraction=self.fast_frames/self.frames if self.frames else 0,
            fast_candidate_fraction=self.fast_frames/self.frames if self.frames else 0,
            fast_candidate_first_detection=first,
            first_detection_latency_from_action_start=first-self.action_start if first is not None and self.action_start is not None else None,
            max_consecutive_fast_frames=self.max_fast_run, track_new=self.new, matched=self.matched,
            short_track_count=short, max_continuous_lifetime=max((t["lifetime_frames"] for t in tracks), default=0),
            track_fragmentation_estimate=short/len(tracks) if tracks else None,
            fragmentation_note="Short-track fraction is an estimate, not hand identity ground truth.",
            diagnostics=["TRACK_MATCHING_MAY_BE_TOO_STRICT"] if self.new > self.matched and short > 2 else [])


TRACK_COLUMNS = "scenario phase frame timestamp track_id track_age_frames consecutive_hits misses centroid_x centroid_y centroid_z distance_history_count raw_distance_history_count point_count raw_distance smoothed_distance robust_range_rate instant_range_rate distance_delta_total consecutive_distance_decreases consecutive_distance_increases raw_doppler_median approach_doppler_median approaching_point_ratio angle angle_stability confirmed confidence legacy_target_selected fast_approach_candidate fast_approach_score fast_approach_reason".split()


def track_csv_row(scenario, phase, frame, timestamp, obj, legacy):
    row = dict(obj, scenario=scenario, phase=phase, frame=frame, timestamp=timestamp,
               track_id=obj.get("track_id", obj.get("id")), legacy_target_selected=bool(legacy and legacy.get("id") == obj.get("id")))
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
        observed = summary["fast_candidate_fraction"] == 0
    return dict(status="PASS" if observed else "WARN", closing_track_count=len(closing), increasing_track_count=len(increasing),
                fast_candidate_fraction=summary["fast_candidate_fraction"], legacy_target_fraction=summary["legacy_target_fraction"],
                scope="Scenario-labelled track evidence only; repeat trials and verify attribution")
