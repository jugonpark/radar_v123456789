"""Fail-closed radar/video decision gate for a future GRISE Pi process.

This module never sends UDP or commands motors. Distances are metres and
speeds are metres/second, matching radar_processing.RadarProcessor output.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GateSettings:
    approach_speed_mps: float = 0.35
    max_range_m: float = 0.90
    max_ttc_s: float = 1.20
    max_radar_age_s: float = 0.25
    max_video_age_s: float = 0.25


@dataclass(frozen=True)
class VideoEvidence:
    timestamp: float
    robot_pose_valid: bool
    calibration_valid: bool
    left_clear: bool
    right_clear: bool
    rear_clear: bool


@dataclass(frozen=True)
class Decision:
    action: str  # MONITOR, STOP, or REQUEST_AVOID
    direction: str | None  # LEFT, RIGHT, REAR; advisory only
    reason: str
    target_id: int | None = None


def decide(radar_result, video: VideoEvidence | None, now: float,
           robot_stopped: bool, settings: GateSettings = GateSettings()) -> Decision:
    """Request avoidance only for a fresh, confirmed, fast approaching target.

    A caller must apply its own motion and safety limits. A missing/invalid
    video state or an unobservable moving-robot radar state yields STOP.
    """
    if not math.isfinite(now) or not robot_stopped:
        return Decision("STOP", None, "robot moving or invalid clock")
    if not isinstance(radar_result, dict):
        return Decision("STOP", None, "radar missing")
    radar_time = radar_result.get("timestamp")
    if not _fresh(radar_time, now, settings.max_radar_age_s):
        return Decision("STOP", None, "radar stale")
    if radar_result.get("processing") != "ACTIVE" or radar_result.get("robot_state") != "MONITORING":
        return Decision("STOP", None, "radar unobservable")
    if video is None or not _fresh(video.timestamp, now, settings.max_video_age_s):
        return Decision("STOP", None, "video missing or stale")
    if not video.calibration_valid or not video.robot_pose_valid:
        return Decision("STOP", None, "video pose or calibration invalid")
    target = radar_result.get("target")
    if target is None:
        return Decision("MONITOR", None, "no confirmed target")
    if not isinstance(target, dict) or not target.get("confirmed") or target.get("confidence") != "HIGH":
        return Decision("STOP", None, "target unconfirmed")
    if target.get("direction_state") != "APPROACHING":
        return Decision("STOP", None, "target direction uncertain")
    distance = target.get("distance")
    speed = target.get("range_rate_velocity")
    ttc = target.get("ttc")
    if not all(_positive_finite(v) for v in (distance, speed, ttc)):
        return Decision("STOP", None, "target metrics invalid", target.get("id"))
    if distance > settings.max_range_m:
        return Decision("MONITOR", None, "target beyond range", target.get("id"))
    if speed < settings.approach_speed_mps or ttc > settings.max_ttc_s:
        return Decision("MONITOR", None, "approach below trigger", target.get("id"))
    # The camera provides clearance; radar bearing alone cannot certify free space.
    for direction, clear in (("REAR", video.rear_clear),
                             ("LEFT", video.left_clear), ("RIGHT", video.right_clear)):
        if clear:
            return Decision("REQUEST_AVOID", direction, "fast approach with video clearance", target.get("id"))
    return Decision("STOP", None, "no verified escape sector", target.get("id"))


def _fresh(timestamp, now, max_age):
    return (isinstance(timestamp, (int, float)) and math.isfinite(timestamp)
            and 0 <= now - timestamp <= max_age)


def _positive_finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value) and value > 0
