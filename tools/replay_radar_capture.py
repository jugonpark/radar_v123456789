"""OFFLINE ALGORITHM EVALUATION. No sensor, serial, telemetry or motor writes."""
import argparse
from dataclasses import asdict
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from radar_processing import RadarProcessor
from raspberry_pi.profiles import settings_for
from tools.radar_track_analysis import TrackAnalysis, TRACK_COLUMNS, track_csv_row, false_positive_diagnostics, scenario_assessment


def load_capture(path, scenarios=None, expected_frame_period=None):
    path = Path(path)
    raw = path / "raw_points.csv" if path.is_dir() else path
    inventory = raw.parent / "frame_diagnostics.csv"
    scene_path = Path(scenarios) if scenarios else raw.parent / "scenarios.csv"
    metadata = {}
    if scene_path.exists():
        with scene_path.open(encoding="utf-8-sig", newline="") as f:
            metadata = {r["scenario"]: r for r in csv.DictReader(f)}
    frames = {}
    warnings = []
    if inventory.exists():
        with inventory.open(encoding="utf-8-sig", newline="") as f:
            for r in csv.DictReader(f):
                key = (r["scenario"], int(r["frame"]))
                frames[key] = dict(scenario=key[0], frame=key[1], timestamp=float(r["received_monotonic"]) if r.get("received_monotonic") else None, points=[])
    else:
        warnings.append("MISSING_FRAME_INVENTORY: empty frames cannot be recovered; fractions may be biased")
    with raw.open(encoding="utf-8-sig", newline="") as f:
        for r in csv.DictReader(f):
            key = (r.get("scenario", "UNLABELLED"), int(r["frame"]))
            timestamp = float(r["received_monotonic"]) if r.get("received_monotonic") else None
            if inventory.exists() and key not in frames:
                raise ValueError(f"Raw frame absent from authoritative inventory: {key}")
            entry = frames.setdefault(key, dict(scenario=key[0], frame=key[1], timestamp=timestamp, points=[]))
            if timestamp is not None and entry["timestamp"] is not None and abs(timestamp-entry["timestamp"]) > 1e-6:
                raise ValueError(f"Raw/inventory timestamp mismatch: {key}")
            entry["points"].append({k:float(r[k]) for k in ("x", "y", "z", "doppler", "snr", "noise") if r.get(k)})
    rows = list(frames.values())
    if any(r["timestamp"] is None for r in rows):
        if expected_frame_period is None or not math.isfinite(expected_frame_period) or expected_frame_period <= 0:
            raise ValueError("Missing receive timestamps: supply positive --expected-frame-period; no silent 10Hz assumption")
        warnings.append("SYNTHETIC_TIMING: explicit expected frame period replaces unavailable timestamps")
        for index, row in enumerate(rows):
            row["timestamp"] = index * expected_frame_period
    rows.sort(key=lambda r:r["timestamp"])
    return rows, metadata, warnings


def replay(rows, metadata, settings, action_window=2.0):
    processors, full, action, timing = {}, {}, {}, {}
    diagnostics = []
    for row in rows:
        name, timestamp = row["scenario"], row["timestamp"]
        if name not in processors:
            processors[name] = RadarProcessor(settings)
            supplied = metadata.get(name, {}).get("action_start_monotonic")
            started = float(supplied) if supplied else timestamp
            timing[name] = dict(action_start_monotonic=started, source="RECORDED_ACTION_START" if supplied else "INFERRED_FIRST_RECEIVE", latency_scope="Scenario START, not actual hand motion onset")
            full[name], action[name] = TrackAnalysis(started), TrackAnalysis(started)
        result = processors[name].process_frame(row["frame"], row["points"], timestamp)
        full[name].add(row["frame"], timestamp, result)
        in_action = 0 <= timestamp-timing[name]["action_start_monotonic"] <= action_window
        if in_action:
            action[name].add(row["frame"], timestamp, result)
        legacy = result.get("legacy_target", result.get("target"))
        diagnostics.extend(track_csv_row(name, "ACTION" if in_action else "HOLD", row["frame"], timestamp, obj, legacy) for obj in result["objects"])
    return [dict(name=name, timing=timing[name], full_capture=full[name].summary(), action_window=action[name].summary(),
                 false_positive_diagnostics=false_positive_diagnostics(name, full[name]), scenario_assessment=scenario_assessment(name, action[name].summary())) for name in full], diagnostics


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--scenarios", type=Path)
    parser.add_argument("--expected-frame-period", type=float)
    parser.add_argument("--profile", choices=["STRICT", "BALANCED", "DIAGNOSTIC"], default="BALANCED")
    parser.add_argument("--action-window", type=float, default=2.0)
    parser.add_argument("--compare-fast-range-rates")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "logs/replay")
    options = {"fast-min-range-rate":("fast_min_range_rate_mps", float), "fast-min-track-frames":("fast_min_track_frames", int),
        "fast-min-total-closing":("fast_min_total_closing_m", float), "fast-required-decrease-frames":("fast_required_decrease_frames", int),
        "fast-max-distance":("fast_max_distance_m", float), "fast-max-angle-jump":("fast_max_angle_jump_deg", float),
        "fast-history-samples":("fast_history_samples", int), "fast-min-points-for-strong-evidence":("fast_min_points_for_strong_evidence", int)}
    for option, (_, kind) in options.items():
        parser.add_argument("--"+option, type=kind)
    args = parser.parse_args(argv)
    if not math.isfinite(args.action_window) or args.action_window <= 0:
        parser.error("action-window must be positive and finite")
    try:
        rows, metadata, warnings = load_capture(args.capture, args.scenarios, args.expected_frame_period)
        settings = settings_for(args.profile)
        for option, (field, _) in options.items():
            value = getattr(args, option.replace("-", "_"))
            if value is not None:
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f"{option} must be positive and finite")
                setattr(settings, field, value)
        evaluated_settings = asdict(settings)
        scenes, diagnostics = replay(rows, metadata, settings, args.action_window)
        comparisons = []
        if args.compare_fast_range_rates:
            for threshold in map(float, args.compare_fast_range_rates.split(",")):
                if not math.isfinite(threshold) or threshold <= 0:
                    raise ValueError("comparison thresholds must be positive and finite")
                settings.fast_min_range_rate_mps = threshold
                compared, _ = replay(rows, metadata, settings, args.action_window)
                comparisons.append(dict(threshold=threshold, action_window_fractions={s["name"]:s["action_window"]["fast_candidate_fraction"] for s in compared}, full_capture_fractions={s["name"]:s["full_capture"]["fast_candidate_fraction"] for s in compared}))
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = dict(validation="OFFLINE ALGORITHM EVALUATION", hardware_status="HARDWARE_TEST_REQUIRED", frames=len(rows), warnings=warnings,
                  profile=args.profile, settings=evaluated_settings, scenarios=scenes, comparisons=comparisons, note="No sensor validation inferred; EXPERIMENTAL HARDWARE_TUNING_REQUIRED; no best threshold selected")
    (args.output_dir/"summary.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    with (args.output_dir/"track_diagnostics.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=TRACK_COLUMNS)
        writer.writeheader()
        writer.writerows(diagnostics)
    lines = [report["validation"], *warnings]
    for scene in scenes:
        for phase in ("full_capture", "action_window"):
            s = scene[phase]
            lines.append(f"{scene['name']} {phase}: legacy_target={s['legacy_target_fraction']:.3%} legacy_high={s['legacy_high_fraction']:.3%} fast={s['fast_candidate_fraction']:.3%} latency={s['first_detection_latency_from_action_start']} timing={scene['timing']['source']}")
        lines.extend(json.dumps(d) for d in scene["false_positive_diagnostics"])
    lines.extend(json.dumps(c) for c in comparisons)
    text = "\n".join(lines)
    (args.output_dir/"test_report.txt").write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
