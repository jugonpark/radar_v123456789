"""Isolated hardware characterization; never sends telemetry or motor commands."""
import argparse
import csv
import json
import math
import statistics
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.radar_track_analysis import TrackAnalysis, TRACK_COLUMNS, track_csv_row, false_positive_diagnostics, scenario_assessment
from radar_processing import RadarProcessor
from raspberry_pi.port_detection import detect_ports
from raspberry_pi.profiles import settings_for
from raspberry_pi.serial_source import RadarSerialSource
from serial.tools import list_ports

SCENARIOS = [
    ("SCENE_0_EMPTY", "Keep the scene still; do not move your hand."),
    ("SCENE_1_STATIC_HAND", "Hold a hand still approximately 0.5-1.0 m in front."),
    ("SCENE_2_SLOW_APPROACH", "Slowly move your hand from approximately 1.2 m to 0.3 m."),
    ("SCENE_3_FAST_APPROACH", "Quickly move your hand from approximately 1.2 m to 0.3 m."),
    ("SCENE_4_RECEDE", "Move your hand away from approximately 0.3 m to 1.2 m."),
    ("SCENE_5_SIDE_MOTION", "Move sideways while keeping distance approximately constant."),
]
STAGES = "raw valid reject_range reject_fov reject_snr snr_missing roi static moving approaching receding unknown_direction clusters single_point_clusters multi_point_clusters track_matched track_new tentative confirmed high_confidence targets".split()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def describe(values):
    values = sorted(v for v in values if finite(v))
    if not values:
        return dict(count=0, min=None, median=None, max=None, p10=None, p50=None, p90=None)
    def percentile(q):
        pos = (len(values) - 1) * q
        lo, hi = math.floor(pos), math.ceil(pos)
        return values[lo] + (values[hi] - values[lo]) * (pos - lo)
    return dict(count=len(values), min=values[0], median=statistics.median(values),
                max=values[-1], p10=percentile(.1), p50=percentile(.5), p90=percentile(.9))


class Capture:
    def __init__(self, name, action_start=None):
        self.name = name
        self.tracks = TrackAnalysis(action_start)
        self.times, self.frames, self.frame_distances = [], [], []
        self.values = {k: [] for k in ("x", "y", "z", "distance", "angle", "doppler", "snr")}
        self.counts = Counter()
        self.raw_counts, self.processor_ms, self.software_ms = [], [], []
        self.range_rates = []
        self.target_frames = self.object_frames = 0
        self.track_ids = set()

    def add(self, parsed, received, result, processor_ms, software_ms):
        self.tracks.add(parsed["frame"], received, result)
        self.times.append(received)
        self.frames.append(parsed["frame"])
        distances = []
        for p in parsed["points"]:
            row = dict(p)
            if all(finite(p.get(k)) for k in ("x", "y", "z")):
                row["distance"] = math.sqrt(sum(p[k] ** 2 for k in ("x", "y", "z")))
                row["angle"] = math.degrees(math.atan2(p["x"], p["y"]))
                distances.append(row["distance"])
            for k in self.values:
                if finite(row.get(k)):
                    self.values[k].append(row[k])
        self.frame_distances.append(statistics.median(distances) if distances else None)
        self.raw_counts.append(len(parsed["points"]))
        self.counts.update({k: result["counts"].get(k, 0) for k in STAGES})
        self.processor_ms.append(processor_ms)
        self.software_ms.append(software_ms)
        self.target_frames += int(result["target"] is not None)
        self.object_frames += bool(result["objects"])
        for o in result["objects"]:
            self.track_ids.add(o["id"])
            if finite(o.get("range_rate_velocity")):
                self.range_rates.append(o["range_rate_velocity"])

    def summary(self, duration):
        n = len(self.frames)
        intervals = [b-a for a, b in zip(self.times, self.times[1:])]
        gaps = duplicates = out_of_order = 0
        for a, b in zip(self.frames, self.frames[1:]):
            delta = (b-a) & 0xffffffff
            if delta == 0:
                duplicates += 1
            elif delta > 0x80000000:
                out_of_order += 1
            else:
                gaps += delta - 1
        pairs = [(t, d) for t, d in zip(self.times, self.frame_distances) if d is not None]
        slope = None
        if len(pairs) > 1:
            mt = statistics.mean(t for t, _ in pairs)
            md = statistics.mean(d for _, d in pairs)
            variance = sum((t-mt)**2 for t, _ in pairs)
            if variance:
                slope = sum((t-mt)*(d-md) for t, d in pairs)/variance
        raw = self.counts["raw"]
        stages = {k: dict(total=self.counts[k], mean_per_frame=self.counts[k]/n if n else 0,
                         per_raw_point=self.counts[k]/raw if raw else None) for k in STAGES}
        rejection = max(("reject_range", "reject_fov", "reject_snr"), key=lambda k: self.counts[k])
        return dict(name=self.name, duration=duration, frames=n, fps=n/duration if duration else 0,
                    inter_frame_seconds=describe(intervals), frame_gaps=gaps, duplicates=duplicates,
                    out_of_order=out_of_order, raw_points_per_frame=describe(self.raw_counts),
                    raw_statistics={k: describe(v) for k, v in self.values.items()}, stages=stages,
                    largest_rejection_stage=rejection if self.counts[rejection] else "NONE",
                    full_capture=self.tracks.summary(), whole_cloud_distance_slope=slope, distance_slope_mps=slope, range_rate_mps=describe(self.range_rates),
                    processor_ms=describe(self.processor_ms), software_ms=describe(self.software_ms),
                    target_frame_fraction=self.target_frames/n if n else 0,
                    object_frame_fraction=self.object_frames/n if n else 0,
                    distinct_track_ids=len(self.track_ids),
                    interpretation="Scene-level raw medians; not identified hand ground truth. Object/track ratios are not point survival probabilities.")


def sign_check(scenarios, approach_sign):
    by_name = {s["name"]: s for s in scenarios}
    fast, recede = by_name.get("SCENE_3_FAST_APPROACH"), by_name.get("SCENE_4_RECEDE")
    if not fast or not recede:
        return dict(status="UNKNOWN", reason="Requires FAST_APPROACH and RECEDE observations")
    def representative(scene, closing):
        tracks = scene.get("action_window", scene.get("full_capture", {})).get("tracks", [])
        usable = [t for t in tracks if t["lifetime_frames"] >= 2 and t["angle_span"] <= 25 and
                  t.get("raw_doppler_median") is not None and abs(t["total_distance_change"]) > .02]
        return (min if closing else max)(usable, key=lambda t:t["total_distance_change"], default=None)
    f, r = representative(fast, True), representative(recede, False)
    if not f or not r:
        return dict(status="UNKNOWN", reason="Insufficient persistent moving track evidence; whole-cloud slope excluded")
    trend_ok = f["total_distance_change"] < 0 and r["total_distance_change"] > 0
    sign_ok = f["raw_doppler_median"]*approach_sign > .1 and r["raw_doppler_median"]*approach_sign < -.1
    return dict(status="PASS" if trend_ok and sign_ok else "WARN", possible_sign_mismatch=trend_ok and not sign_ok,
                reason="Track range trend plus object Doppler; track identity is not labelled hand ground truth", fast_track=f, recede_track=r)



def classify(stream, scenarios, expected_fps, tolerance, cli_ok, port_ok, approach_sign):
    errors = sum(stream.get(k, 0) for k in ("parse_failures", "invalid_lengths", "point_count_mismatches", "truncated_tlv_envelopes"))
    frames = sum(s["frames"] for s in scenarios)
    duration = sum(s["duration"] for s in scenarios)
    fps = frames/duration if duration else 0
    continuity = sum(s["frame_gaps"] + s["duplicates"] + s["out_of_order"] for s in scenarios)
    sign = sign_check(scenarios, approach_sign)
    stream_status = "FAIL" if not frames else "WARN" if errors or stream.get("resyncs", 0) else "PASS"
    stability = "PASS" if frames and not continuity and abs(fps-expected_fps) <= expected_fps*tolerance else "WARN"
    raw_count = sum(s["raw_statistics"]["distance"]["count"] for s in scenarios)
    indexed = {s["name"]: s for s in scenarios}
    approach_names = ("SCENE_2_SLOW_APPROACH", "SCENE_3_FAST_APPROACH")
    quiet_names = ("SCENE_0_EMPTY", "SCENE_1_STATIC_HAND", "SCENE_4_RECEDE", "SCENE_5_SIDE_MOTION")
    processing_ok = (sign["status"] == "PASS" and all(name in indexed for name, _ in SCENARIOS)
                     and all(indexed[name]["target_frame_fraction"] > 0 for name in approach_names)
                     and all(indexed[name]["target_frame_fraction"] == 0 for name in quiet_names))
    processing = "PASS" if processing_ok else "WARN"
    sensor = "HARDWARE_PROBLEM" if not cli_ok or not port_ok or not frames else "READY" if stream_status == stability == sign["status"] == "PASS" and raw_count else "NEEDS_RETEST"
    return dict(fps=fps, frame_count=frames, duration=duration, approach_sign_check=sign,
                checks={"USB/PORT": "PASS" if port_ok else "FAIL", "CLI CONFIG": "PASS" if cli_ok else "FAIL",
                        "DATA STREAM": stream_status, "FRAME STABILITY": stability,
                        "TLV PARSE": "PASS" if frames and not errors else "WARN" if frames else "UNKNOWN",
                        "RAW POINT RESPONSE": "PASS" if raw_count and sign["status"] == "PASS" else "WARN" if raw_count else "FAIL",
                        "DOPPLER DIRECTION": sign["status"], "DISTANCE TREND": sign["status"],
                        "POST PROCESSING": processing, "TARGET RELIABILITY": "WARN"},
                sensor_status=sensor, processing_status="READY" if processing_ok else "TUNING_REQUIRED",
                note="Raw presence is not hand response validation. Target reliability requires repeated labelled trials; a single target is insufficient.")


class Deadline:
    def __init__(self, seconds):
        self.end = time.monotonic() + seconds
    def is_set(self):
        return time.monotonic() >= self.end


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli-port")
    parser.add_argument("--data-port")
    parser.add_argument("--cfg", type=Path, default=ROOT / "profile_3d_aop.cfg")
    parser.add_argument("--profile", choices=["BALANCED", "STRICT", "DIAGNOSTIC"], default="BALANCED")
    parser.add_argument("--duration", type=float, default=30, help="Seconds per scenario")
    parser.add_argument("--expected-fps", type=float, help="Defaults to frameCfg period in CFG")
    parser.add_argument("--fps-tolerance", type=float, default=.2)
    parser.add_argument("--action-window", type=float, default=2.0)
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--raw-log", action="store_true")
    parser.add_argument("--diag-log", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "logs/hardware_test")
    args = parser.parse_args(argv)
    if args.action_window <= 0 or not math.isfinite(args.action_window) or args.duration <= 0 or not math.isfinite(args.duration) or not 0 <= args.fps_tolerance < 1:
        parser.error("duration must be finite and positive; tolerance must be in [0,1)")
    expected = args.expected_fps
    if expected is None:
        for line in args.cfg.read_text().splitlines():
            fields = line.split()
            if fields and fields[0] == "frameCfg" and len(fields) > 5:
                expected = 1000/float(fields[5])
    if expected is None or not finite(expected) or expected <= 0:
        parser.error("Provide --expected-fps when CFG has no positive frame period")
    output = args.output_dir / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True)
    ports = detect_ports(cli_override=args.cli_port, data_override=args.data_port)
    metadata = []
    available = list(list_ports.comports())
    for role, device, baud in (("CLI", ports.cli, 115200), ("DATA", ports.data, 921600)):
        p = next((p for p in available if p.device == device or Path(p.device).resolve() == Path(device).resolve()), None)
        info = {k: getattr(p, k, None) for k in ("vid", "pid", "description", "interface", "manufacturer")}
        info.update(role=role, device=device, baud=baud, mapping=ports.source)
        metadata.append(info)
        print("[RADAR HW]", json.dumps(info))
    print("Interface labels missing => mapping is unverified, not inferred PASS. No UDP transmission.")
    same_port = Path(ports.cli).resolve() == Path(ports.data).resolve()
    source = RadarSerialSource(ports.cli, ports.data, args.cfg)
    settings = settings_for(args.profile)
    processor = RadarProcessor(settings)
    handles = []
    def writer(name, columns):
        handle = (output/name).open("w", newline="", encoding="utf-8")
        handles.append(handle)
        w = csv.writer(handle)
        w.writerow(columns)
        return w
    objects = writer("objects.csv", ["scenario", "frame", "received_monotonic", "object_json"])
    scenes = writer("scenarios.csv", ["scenario", "start_wall", "duration", "frames", "action_start_wall", "action_start_monotonic", "action_window"])
    tracks = writer("track_diagnostics.csv", TRACK_COLUMNS)
    raw_columns = ["x", "y", "z", "distance", "angle", "doppler", "snr", "noise"]
    raw = writer("raw_points.csv", ["scenario", "frame", "received_monotonic", *raw_columns]) if args.raw_log else None
    diag = writer("frame_diagnostics.csv", ["scenario", "frame", "received_monotonic", *STAGES, "processor_ms"]) if args.diag_log else None
    results = []
    all_capture = Capture("ALL_SCENARIOS")
    all_intervals = []
    cli_ok = port_ok = False
    failure = None
    try:
        if same_port:
            raise ValueError("CLI and DATA must be different serial interfaces")
        source.configure()
        cli_ok = port_ok = True
        print("CLI_CONFIG: PASS", Counter(r["status"] for r in source.command_results), "sent_commands=", len(source.command_results))
        for name, instructions in SCENARIOS if args.interactive else [("UNLABELLED", "Observe current scene")]:
            if args.interactive:
                input(f"\n{name}: {instructions}\nPress Enter when ready: ")
                for number in (3, 2, 1):
                    print(number, flush=True)
                    time.sleep(1)
            source.data.reset_input_buffer()
            processor.reset()
            start_wall = datetime.now().isoformat()
            started = last_print = time.monotonic()
            print("ACTION_START", start_wall, started, flush=True)
            capture = Capture(name, started)
            action = Capture(name, started)
            try:
                for parsed, received in source.frames(Deadline(args.duration)):
                    before = time.monotonic()
                    result = processor.process_frame(parsed["frame"], parsed["points"], received)
                    finished = time.monotonic()
                    proc_ms = (finished-before)*1000
                    software_ms = proc_ms + source.parse_seconds[-1]*1000
                    capture.add(parsed, received, result, proc_ms, software_ms)
                    all_capture.add(parsed, received, result, proc_ms, software_ms)
                    in_action = 0 <= received-started <= args.action_window
                    if in_action:
                        action.add(parsed, received, result, proc_ms, software_ms)
                    legacy = result.get("legacy_target", result.get("target"))
                    for obj in result["objects"]:
                        row = track_csv_row(name, "ACTION" if in_action else "HOLD", parsed["frame"], received, obj, legacy)
                        tracks.writerow([row[k] for k in TRACK_COLUMNS])
                    for obj in result["objects"]:
                        objects.writerow([name, parsed["frame"], received, json.dumps({k:v for k,v in obj.items() if k not in ("distance_history", "raw_distance_history")}, allow_nan=False)])
                    if raw:
                        for p in parsed["points"]:
                            row = dict(p)
                            if all(finite(p.get(k)) for k in ("x", "y", "z")):
                                row["distance"] = math.sqrt(sum(p[k]**2 for k in ("x", "y", "z")))
                                row["angle"] = math.degrees(math.atan2(p["x"], p["y"]))
                            raw.writerow([name, parsed["frame"], received, *[row.get(k) for k in raw_columns]])
                    if diag:
                        diag.writerow([name, parsed["frame"], received, *[result["counts"].get(k, 0) for k in STAGES], proc_ms])
                    if finished-last_print >= 1:
                        last_print = finished
                        print(f"[HW] scene={name} frame={parsed['frame']} raw={len(parsed['points'])} target={int(result['target'] is not None)}")
                        if args.verbose:
                            print({k: result["counts"].get(k, 0) for k in STAGES})
            finally:
                duration = time.monotonic()-started
                summary = capture.summary(duration)
                summary["action_window"] = action.tracks.summary()
                summary["scenario_assessment"] = scenario_assessment(name, summary["action_window"])
                summary["timing"] = dict(action_start_wall=start_wall, action_start_monotonic=started, source="RECORDED_ACTION_START", latency_scope="Scenario START, not actual hand motion onset")
                summary["false_positive_diagnostics"] = false_positive_diagnostics(name, capture.tracks)
                moving = [d for d in capture.values["doppler"] if abs(d) > settings.doppler_deadband_mps]
                summary["moving_doppler_median"] = statistics.median(moving) if moving else None
                results.append(summary)
                all_intervals.extend(b-a for a, b in zip(capture.times, capture.times[1:]))
                scenes.writerow([name, start_wall, duration, summary["frames"], start_wall, started, args.action_window])
    except (Exception, KeyboardInterrupt) as exc:
        failure = f"{type(exc).__name__}: {exc}"
        print("TEST INTERRUPTED/FAILED:", failure)
    finally:
        cli_ok = source.configuration_complete
        source.close()
        for h in handles:
            h.close()
    report = classify(source.stream_stats, results, expected, args.fps_tolerance, cli_ok, port_ok, settings.approach_sign)
    labels_verified = all((word in ((m["description"] or "") + " " + (m["interface"] or "")).lower())
                          for m, word in zip(metadata, ("enhanced", "standard")))
    if port_ok and not labels_verified:
        report["checks"]["USB/PORT"] = "WARN"
        report["sensor_status"] = "NEEDS_RETEST"
    if cli_ok and any(r["status"] == "PROMPT" for r in source.command_results):
        report["checks"]["CLI CONFIG"] = "WARN"
        report["sensor_status"] = "NEEDS_RETEST"
    report.update(timestamp=datetime.now().isoformat(), cfg=str(args.cfg), profile=args.profile,
                  ports=metadata, interface_labels_verified=labels_verified,
                  expected_fps=expected, fps_tolerance=args.fps_tolerance,
                  stream=source.stream_stats, parse_ms=describe([v*1000 for v in source.parse_seconds]),
                  cli_commands=source.command_results, scenarios=results, failure=failure,
                  raw_statistics={k: describe(v) for k, v in all_capture.values.items()},
                  inter_frame_seconds=describe(all_intervals),
                  processor_ms=describe(all_capture.processor_ms), software_ms=describe(all_capture.software_ms),
                  frame_gaps=sum(s["frame_gaps"] for s in results),
                  parse_errors=source.stream_stats.get("parse_failures", 0),
                  latency_scope="Host parser + RadarProcessor only, excludes RF capture/UART transfer/logging; intervals measure host delivery.")
    if failure and (failure.startswith("KeyboardInterrupt") or report["frame_count"]):
        report["sensor_status"] = "NEEDS_RETEST"
    report["fast_approach_check"] = dict(status="PASS" if any(s["name"] == "SCENE_3_FAST_APPROACH" and s["action_window"]["fast_candidate_fraction"] > 0 for s in results) and not any(s["false_positive_diagnostics"] for s in results) else "WARN", reason="Track candidate evidence within recorded action window; requires repeated labelled hardware trials", scenarios={s["name"]:dict(fast_fraction=s["action_window"]["fast_candidate_fraction"], legacy_fraction=s["action_window"]["legacy_target_fraction"]) for s in results})
    report["checks"]["FAST APPROACH"] = report["fast_approach_check"]["status"]
    report["check_reasons"] = {
        "FAST APPROACH": report["fast_approach_check"],
        "USB/PORT": dict(interface_labels_verified=labels_verified, mapping=ports.source),
        "CLI CONFIG": dict(prompt_only_ack=sum(r["status"] == "PROMPT" for r in source.command_results), sent_commands=len(source.command_results), configuration_complete=cli_ok),
        "DATA STREAM": {k:source.stream_stats.get(k, 0) for k in ("parsed_frames", "invalid_lengths", "resyncs", "parse_failures", "point_count_mismatches", "truncated_tlv_envelopes")},
        "TLV PARSE": {k:source.stream_stats.get(k, 0) for k in ("parse_failures", "point_count_mismatches", "truncated_tlv_envelopes")},
        "FRAME STABILITY": dict(expected_fps=expected, measured_fps=report["fps"], gaps=report["frame_gaps"], duplicates=sum(s["duplicates"] for s in results), out_of_order=sum(s["out_of_order"] for s in results))}
    text = "=== RADAR HARDWARE VALIDATION ===\n" + "\n".join(f"{k:22} {v}\nreason: {json.dumps(report['check_reasons'].get(k, {'assessment':report['approach_sign_check']}))}" for k, v in report["checks"].items())
    text += f"\nSENSOR STATUS: {report['sensor_status']}\nPROCESSING STATUS: {report['processing_status']}\n"
    for scene in results:
        for phase in ("full_capture", "action_window"):
            text += f"{scene['name']} {phase}: {json.dumps(scene[phase])}\n"
        for evidence in scene["false_positive_diagnostics"]:
            text += json.dumps(evidence) + "\n"
    text += json.dumps(report, indent=2, allow_nan=False)
    (output/"summary.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    (output/"test_report.txt").write_text(text, encoding="utf-8")
    print(text.split("\n{")[0], "\nResults:", output)
    for scene in results:
        print(f"\n{scene['name']}: frames={scene['frames']} fps={scene['fps']:.2f} largest_rejection={scene['largest_rejection_stage']}")
        for stage, stats in scene["stages"].items():
            ratio = stats["per_raw_point"]
            print(f"{stage:22} mean/frame={stats['mean_per_frame']:.3f} per_raw={'N/A' if ratio is None else format(ratio, '.3%')}")
    return 1 if failure or not report["frame_count"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
