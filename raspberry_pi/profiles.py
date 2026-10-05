"""Pi-only sensitivity presets; Windows Settings defaults remain unchanged."""
from radar_processing import Settings


def settings_for(profile):
    name = profile.upper()
    if name == "STRICT":
        return Settings()
    if name == "BALANCED":
        return Settings(max_range_m=1.50, threat_max_range_m=.90,
                        min_snr_db=8.0, min_cluster_points=1,
                        temporal_required=2, distance_smoothing_alpha=.60,
                        track_match_distance_m=.35, track_match_angle_deg=25.0,
                        max_track_speed_mps=3.0, target_requires_threat=True,
                        cluster_before_direction=True, fast_path_mode="OBSERVE",
                        fast_confidence_on_two_frames=True)
    if name == "DIAGNOSTIC":
        return Settings(max_range_m=1.50, threat_max_range_m=.90,
                        min_snr_db=0, require_snr=False, min_cluster_points=1,
                        temporal_required=2, diagnostic_only=True,
                        cluster_before_direction=True, fast_path_mode="OBSERVE")
    raise ValueError(f"Unknown sensitivity profile: {profile}")
