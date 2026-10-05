# Approach Speed Decision

**EXPERIMENTAL — HARDWARE_TUNING_REQUIRED**. This independent OFF/OBSERVE path reports a decision flag. It does not change legacy target/risk, Fast Path V2, grise, firmware or motors. OBSERVE is default in BALANCED/DIAGNOSTIC; STRICT defaults OFF.

Approach speed reuses the existing median of all pairwise `(older_distance-newer_distance)/(newer_time-older_time)` rates in the recent raw-distance window (default 5 samples). Positive means closing; negative means retreating. No duplicate estimator or Kalman filter is introduced. At least 3 consecutive observations **and 3 actual samples** are needed. History shorter than this remains UNKNOWN. Misses, frame gaps and track changes reset evidence.

Observe distance is `min(max_range_m,1.2m)`; avoid distance starts at 0.8m. Radial speed deadband is 0.10m/s; fast speed starts at 0.50m/s. These are configurable experimental distance-history thresholds, not Doppler resolution. States: OUT_OF_RANGE, IN_RANGE_STATIC, RECEDING, SLOW_APPROACH, FAST_APPROACH, UNKNOWN.

AVOID_CANDIDATE requires a persistent current track, net negative distance change, at least two consecutive decreases with the default minimum of 3 frames, fast positive closing rate and distance <= avoid threshold. The configurable decrease requirement is `min(2, approach_min_track_frames - 1)`: setting the minimum to 2 frames permits one decrease, while the default 3-frame minimum requires two. A single latest-distance spike is insufficient under the default configuration. TTC = distance/speed only for positive speed, provided diagnostically. Doppler supports confidence or records disagreement without vetoing clear closing history. Existing ROI/SNR/MOVING Doppler-deadband filtering still occurs before clustering; objects removed there cannot be recovered by this path. STRICT's earlier pipeline remains unchanged; use BALANCED for this experiment.

Primary object priority: avoid candidate, farther fast approach, slow approach, other usable states; ties use TTC, distance, Track ID. Frame decisions SAFE/WATCH/AVOID_CANDIDATE are diagnostic. `current_frame_valid=false` means no usable approach measurement (e.g. OFF, empty, paused, all UNKNOWN); SAFE in this case is **not verified safety** and this field is separate from sensor health.

## Windows CMD: actual next trial

Close the existing radar GUI/runtime to release COM ports. Connect the radar, then:

```cmd
cd /d "C:\Users\jugon\Documents\카카오톡 받은 파일\radar"
python tools\pi_radar_hardware_check.py ^
  --cli-port COM5 ^
  --data-port COM3 ^
  --profile BALANCED ^
  --approach-test ^
  --duration 6 ^
  --action-window 2 ^
  --raw-log ^
  --diag-log ^
  --verbose
```

This forces interactive OBSERVE and runs EMPTY, APPROACH, RECEDE, SIDE. Without `--approach-test`, the existing six-scene interactive test remains available. Prepare each scene, press Enter, wait 3/2/1, then perform one movement immediately after ACTION_START. Full six-second capture continues; first two seconds are analyzed separately.

1. EMPTY: keep the scene still.
2. APPROACH: approximately 1.0–1.2m to 0.3m once, then hold.
3. RECEDE: approximately 0.3m to 1.0–1.2m once, then hold.
4. SIDE: maintain approximately 0.6m radial distance, move sideways.

First use a metal tumbler or metal plate with strong radar reflection. Then repeat the same geometry with a hand. `[APPROACH]` prints one primary object at most 5Hz; frame CSV is authoritative for brief transients. It displays signed speed, distance, state and avoid flag. No point console spam. Exit Ctrl+C; an interrupted capture cannot receive an overall PASS.

Pi equivalent:

```bash
python tools/pi_radar_hardware_check.py --cli-port /dev/ttyUSB0 --data-port /dev/ttyUSB1 --profile BALANCED --approach-test --duration 6 --action-window 2 --raw-log --diag-log --verbose
```

Settings overrides for checker/replay: `--approach-fast-speed 0.5`, `--approach-avoid-distance 0.8`, `--approach-observe-distance 1.2`, `--approach-min-track-frames 3`, `--approach-speed-deadband 0.1`, `--approach-path-mode OFF|OBSERVE`. The simple test forces OBSERVE even if OFF was requested.

## Replay and measurement definitions

```cmd
python tools\replay_radar_capture.py logs\hardware_test\20261005_154630_927421 --compare-approach-fast-speeds 0.3,0.5,0.7 --compare-approach-avoid-distances 0.6,0.8 --output-dir logs\approach_replay
python tools\replay_radar_capture.py logs\hardware_test\20261005_154630_927421 --approach-fast-speed 0.7 --approach-avoid-distance 0.6 --output-dir logs\approach_replay_tuned
```

Pi uses the same commands with `/` path separators. Old Fast V2 `--compare-fast-range-rates` remains available. Comparisons do not select a best threshold.

Full/action summaries and scalar track CSV contain approach results. `approach_decision_frame_fraction` means WATCH or AVOID frames; FAST fraction counts frames with any current new-path FAST object; avoid fraction counts frame avoid flags. Signed max/median speeds and minimum distances use usable primary objects, excluding missing/UNKNOWN fillers. Representative primary Track ID is the most frequent ID with smallest-ID tie break; all selected IDs are listed. First-fast/avoid latency is from scenario START, **not measured physical motion onset**. Old captures without ACTION_START explicitly use INFERRED_FIRST_RECEIVE. Actual receive timestamps and authoritative inventory preserve duplicate/zero-point receive events; missing timing is never silently treated as 10Hz.

Reports include coverage, incomplete captures, empty action windows and numeric scene-inconsistent fast/avoid evidence for EMPTY/STATIC/RECEDE/SIDE. Overall approach PASS requires all four complete nonempty scenes and positive approach action evidence, observed RECEDING action evidence, and no contradictions. The old six-scene Fast V2 overall check remains UNKNOWN during the four-scene test because STATIC/SLOW are missing; the new independent four-scene approach check has separate coverage. No transport verdict is inferred from replay.

## Existing capture: software-only result

OFFLINE ALGORITHM EVALUATION of existing 590 frames; no new hardware run. Default full-capture frame fractions:

| Scene | Legacy target | Fast V2 candidate | Approach FAST | Approach avoid |
|---|---:|---:|---:|---:|
| EMPTY | 13.000% | 3.000% | 4.000% | 4.000% |
| STATIC | 8.163% | 0% | 0% | 0% |
| SLOW | 12.121% | 0% | 1.010% | 0% |
| FAST | 11.340% | 2.062% | 2.062% | 2.062% |
| RECEDE | 14.286% | 5.102% | 9.184% | 9.184% |
| SIDE | 12.245% | 0% | 3.061% | 3.061% |

In inferred first-two-second action windows, approach FAST/avoid is 0% except RECEDE 5.263%; legacy target is EMPTY5%, STATIC0%, SLOW0%, FAST0%, RECEDE10.526%, SIDE14.286%. Fast V2 action fractions are all 0%. FAST first detection is ~4.121s after inferred receive start; RECEDE first detection ~0.802s. These results **do not demonstrate improved discrimination or successful hand detection**. Do not enable actuation or settle final thresholds from this dataset. Capture ACTION_START-labelled trials before tuning.

`SOFTWARE_VERIFIED`: deterministic unit/regression, mock checker/runtime and offline replay. `HARDWARE_TEST_REQUIRED`: actual reflection/hand tracking, accuracy, timing and repeated scene separation. No hardware success claim follows from synthetic tests.
