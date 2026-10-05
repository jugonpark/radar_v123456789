"""Receive radar telemetry independently of robot control."""
import argparse
import json
import math
import socket
import time

HEALTH = {"CONNECTING", "CONFIGURING", "OK", "STALE", "DISCONNECTED", "CONFIG_ERROR", "ERROR"}


def decode_packet(data):
    def reject_constant(value):
        raise ValueError(f"Non-finite JSON: {value}")
    p = json.loads(data, parse_constant=reject_constant)
    if not isinstance(p, dict) or type(p.get("protocol_version")) is not int or p["protocol_version"] != 1:
        raise ValueError("Unsupported telemetry schema/version")
    if p.get("radar_health") not in HEALTH or not isinstance(p.get("radar_valid"), bool):
        raise ValueError("Invalid health/validity")
    if not isinstance(p.get("frame"), int) or isinstance(p["frame"], bool) or p["frame"] < 0:
        raise ValueError("Invalid frame")
    if not isinstance(p.get("timestamp"), (float, int)) or isinstance(p["timestamp"], bool) or not math.isfinite(p["timestamp"]):
        raise ValueError("Invalid timestamp")
    target = p.get("target")
    if target is not None:
        if not isinstance(target, dict):
            raise ValueError("Invalid target")
        for k in ("id", "distance", "angle", "doppler_velocity", "range_rate_velocity", "ttc"):
            v = target.get(k)
            if v is not None and (not isinstance(v, (float, int)) or isinstance(v, bool) or not math.isfinite(v)):
                raise ValueError(f"Invalid target {k}")
    return p


def safe_view(packet, receive_age, wall_age, stale_seconds):
    valid = (packet["radar_health"] == "OK" and packet["radar_valid"]
             and 0 <= receive_age <= stale_seconds and -stale_seconds <= wall_age <= stale_seconds)
    return dict(radar_health=packet["radar_health"], frame=packet["frame"],
                receive_age_s=receive_age, sender_wall_age_s=wall_age,
                radar_valid=valid, target=packet.get("target") if valid else None,
                risk=(packet.get("target") or {}).get("risk", "N/A") if valid else "N/A")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--stale-seconds", type=float, default=1.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.stale_seconds) or args.stale_seconds <= 0:
        parser.error("stale-seconds must be finite and positive")
    malformed = received = 0
    last = packet = None
    start = printed = time.monotonic()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(.2)
    print(f"UDP probe {args.host}:{args.port}; sender wall age requires synchronized clocks. Ctrl+C to stop.")
    try:
        while True:
            try:
                data, peer = sock.recvfrom(65535)
                candidate = decode_packet(data)
                packet, last = candidate, time.monotonic()
                received += 1
            except socket.timeout:
                pass
            except (ValueError, UnicodeDecodeError, TypeError):
                malformed += 1
            now = time.monotonic()
            if now - printed >= 1:
                printed = now
                view = safe_view(packet, now-last, time.time()-packet["timestamp"], args.stale_seconds) if packet else dict(radar_valid=False, target=None, risk="N/A", radar_health="NO_PACKET")
                print(json.dumps(dict(packet_rate_hz=received/(now-start), malformed=malformed, **view)))
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
