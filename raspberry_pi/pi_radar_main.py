import argparse
import logging
import signal
import time
from pathlib import Path

try:
    from .pi_config import PiConfig, ROOT
    from .runtime import RadarRuntime
except ImportError:  # direct `python raspberry_pi/pi_radar_main.py`
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from pi_config import PiConfig, ROOT
    from runtime import RadarRuntime


def build_parser():
    p = argparse.ArgumentParser(description="IWR6843 Raspberry Pi headless radar runtime")
    p.add_argument("--cli-port"); p.add_argument("--data-port"); p.add_argument("--cfg", default=str(ROOT / "profile_3d_aop.cfg"))
    p.add_argument("--no-auto-port", action="store_true"); p.add_argument("--log-dir", default=str(ROOT / "logs"))
    p.add_argument("--telemetry-host", default="127.0.0.1"); p.add_argument("--telemetry-port", type=int, default=8890)
    p.add_argument("--telemetry-hz", type=float, default=10.0); p.add_argument("--no-telemetry", action="store_true")
    p.add_argument("--raw-log", action="store_true"); p.add_argument("--verbose", action="store_true")
    p.add_argument("--diag-log", action="store_true", help="write one diagnostic CSV row per frame")
    p.add_argument("--profile", choices=("BALANCED", "STRICT", "DIAGNOSTIC"), default="BALANCED")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(message)s")
    config = PiConfig(cfg_path=Path(args.cfg), log_dir=Path(args.log_dir), telemetry_host=args.telemetry_host,
                      telemetry_port=args.telemetry_port, telemetry_hz=args.telemetry_hz, raw_logging=args.raw_log)
    runtime = RadarRuntime(config, no_auto_port=args.no_auto_port, cli_override=args.cli_port, data_override=args.data_port,
                           telemetry_enabled=not args.no_telemetry, profile=args.profile,
                           diagnostic_logging=args.diag_log)
    print(f"GRISE Radar Pi | profile={args.profile} CFG={config.cfg_path} CLI={runtime.cli_port} DATA={runtime.data_port} source={runtime.port_source} telemetry={not args.no_telemetry} raw={args.raw_log}")
    stopping = {"value": False}
    def stop_handler(_signum, _frame):
        stopping["value"] = True
    signal.signal(signal.SIGINT, stop_handler)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, stop_handler)
    try:
        runtime.connect()
        last_report = time.monotonic()
        while not stopping["value"]:
            try:
                for parsed, received in runtime.source.frames(_StopFlag(stopping)):
                    result = runtime.process(parsed, received)
                    now = time.monotonic()
                    if now - last_report >= 1.0:
                        target = 1 if result.get("target") else 0
                        c = result["counts"]
                        print(f"[RADAR] FPS={runtime.frame_count / max(now-runtime.start_monotonic, 1e-6):.1f} frame={parsed['frame']} health={runtime.health} raw={c['raw']} roi={c['roi']} moving={c['moving']} approach={c['approaching']} clusters={c['clusters']} tentative={c['tentative']} confirmed={c['confirmed']} target={target} risk={result['target']['risk'] if result.get('target') else 'N/A'}")
                        if args.verbose:
                            print(f"[RADAR DIAG] range_drop={c['reject_range']} fov_drop={c['reject_fov']} snr_drop={c['reject_snr']} snr_missing={c['snr_missing']} snr_min/med/max={c['snr_min']}/{c['snr_median']}/{c['snr_max']} static={c['static']} recede={c['receding']} single={c['single_point_clusters']} matched={c['track_matched']} new={c['track_new']} high={c['high_confidence']}")
                        last_report = now
                runtime.check_health()
            except Exception as exc:
                runtime.health = "DISCONNECTED"
                runtime.processor.reset()
                print(f"[RADAR] DISCONNECTED: {exc}")
                if runtime.source:
                    runtime.source.close()
                    runtime.source = None
                if stopping["value"]:
                    break
                time.sleep(config.reconnect_interval_s)
                runtime.connect()
                print(f"[RADAR] reconnected CLI={runtime.cli_port} DATA={runtime.data_port}")
    except Exception as exc:
        runtime.health = "ERROR"
        print(f"[RADAR] ERROR {exc}")
        raise
    finally:
        runtime.close()


class _StopFlag:
    def __init__(self, state): self.state = state
    def is_set(self): return self.state["value"]


if __name__ == "__main__":
    main()
