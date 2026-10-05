# Track Fast Path V2

Status: **EXPERIMENTAL — HARDWARE_TUNING_REQUIRED**. This is radar subsystem work; no grise, ESP32 firmware, camera fusion or motor commands.

The processor preserves the baseline target as `legacy_target`. BALANCED/DIAGNOSTIC compute diagnostic evidence in OBSERVE mode. STRICT defaults OFF. OBSERVE never promotes candidates to target or changes baseline risk. ENABLED explicitly permits experimental target selection and must be separately hardware tested.

Moving points are spatially clustered before track diagnostics. Recent bounded raw distance samples use median pairwise closing rates; positive rate means decreasing distance. Persistence, cumulative closing, repeated decreases, rate, relevant distance and angle stability form required evidence. Multi-point support and approach Doppler add evidence. Doppler disagreement is recorded, not a sole hard rejection. Raw signed Doppler and `approach_sign * raw_doppler` are separate fields. EMA remains display/baseline smoothing.

## Windows actual next test

Close any other reader of the same COM ports, then CMD:

```cmd
cd /d "C:\Users\jugon\Documents\카카오톡 받은 파일\radar"
python tools\pi_radar_hardware_check.py --cli-port COM5 --data-port COM3 --interactive --duration 10 --action-window 2 --raw-log --diag-log
```

Prepare each of the six scenes, press Enter, wait 3/2/1, then perform the motion once immediately after ACTION_START. Full capture continues during the hold interval. ACTION_START is host scenario timing, not a measured hand motion onset.

Pi:

```bash
python tools/pi_radar_hardware_check.py --cli-port /dev/ttyUSB0 --data-port /dev/ttyUSB1 --interactive --duration 10 --action-window 2 --raw-log --diag-log
```

Checker retains raw whole-cloud statistics as supplemental diagnostics. Sign assessment uses persistent moving track range change and object Doppler; whole-cloud median slope does not determine sign or fast approach. Track representatives are observations, not identified hands. Short-track fraction is a fragmentation estimate, not ground truth.

Output keeps existing files and adds scalar `track_diagnostics.csv`. Object JSON strips full history arrays. Track summaries include lifetime/distance/rates/Doppler/angle/confidence/candidate counts, longest/largest-closing/fastest tracks and scenario-relative first detection latency. Quiet, retreat and side scenes record numeric FAST_FALSE_POSITIVE evidence. Reports show numeric warning reasons immediately below statuses.

## Replay

```cmd
python tools\replay_radar_capture.py logs\hardware_test\20261005_154630_927421 --profile BALANCED --compare-fast-range-rates 0.3,0.5,0.7,1.0
python tools\replay_radar_capture.py logs\hardware_test\20261005_154630_927421 --fast-min-range-rate 0.5 --fast-min-track-frames 3 --output-dir logs\replay_tuned
```

```bash
python tools/replay_radar_capture.py logs/hardware_test/20261005_154630_927421 --profile BALANCED --compare-fast-range-rates 0.3,0.5,0.7,1.0
```

Input may also be `raw_points.csv --scenarios scenarios.csv`. `frame_diagnostics.csv` beside raw data is authoritative frame inventory, including zero-point frames. Real receive times determine dt. Missing timing requires explicit `--expected-frame-period 0.1`; this is labelled synthetic timing. Missing inventory warns that empty frames cannot be recovered. Older captures without ACTION_START use explicitly labelled INFERRED_FIRST_RECEIVE timing. Threshold comparisons contain both full and action-window fractions and never select a best threshold.

Replay is **OFFLINE ALGORITHM EVALUATION**, not transport/sensor validation. Results are software evidence only; no sensor READY status is synthesized. A single labelled trial cannot establish accuracy or final thresholds. `SOFTWARE_VERIFIED` and `HARDWARE_TEST_REQUIRED` remain separate.

## Existing capture software-only result

Replay of `20261005_154630_927421` preserves 590 frames via frame diagnostics. No new hardware test was run.

| Scene | Legacy target frames | Legacy HIGH frames | Fast candidate frames |
|---|---:|---:|---:|
| EMPTY | 13.000% | 18.000% | 3.000% |
| STATIC | 8.163% | 13.265% | 0% |
| SLOW | 12.121% | 18.182% | 0% |
| FAST | 11.340% | 13.402% | 2.062% |
| RECEDE | 14.286% | 21.429% | 5.102% |
| SIDE | 12.245% | 16.327% | 0% |

Fractions count frames with at least one selected baseline target, HIGH object, or fast candidate; they are not mean object counts per frame. All inferred first-two-second action windows have zero fast candidates. FAST first candidate is approximately 4.121 seconds after inferred first receive, not measured motion onset. Default settings **do not establish improved scenario separation**; EMPTY/RECEDE retain scenario-inconsistent candidate evidence. Repeated ACTION_START-labelled capture is required. Do not promote ENABLED or select a final threshold from this dataset.

The original capture transport summary reported resyncs=9, truncated_tlv_envelopes=7, invalid_lengths=0, parse_failures=0 and prompt-only CLI acknowledgement(s). These are recorded numeric diagnostics, not evidence for changing packet format. V2 live reports explain such WARNs explicitly.
