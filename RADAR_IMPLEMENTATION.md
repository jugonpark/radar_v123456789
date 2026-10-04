# Stationary robot radar processing

Run `python radar_gui.py`. The original `radar_gui_v1_ready.py` remains available. No firmware, motor command, or ESP32 telemetry changes were made.

## Existing code analysis and problems

- `radar_receiver.py` and `radar_receiver_robot_v1.py` configure COM5/COM3 and parse the TI mmWave demo header, detected point TLV 1, and side info TLV 7. The robot receiver makes risk decisions from individual points.
- `radar_gui_v1_ready.py` has a serial worker, `queue.Queue` handoff, COM/CFG selectors, a Tk radar map, and point CSV logging. Its old target is the point with minimum TTC; it has no object clustering or temporal confirmation.
- `radar_gui_log_20261005_002840.csv` has one row per point with `timestamp_s`, `frame_id`, x/y/z, Doppler and SNR. It records no rows for empty frames. Do not use its old point `risk` or `is_candidate` as object ground truth.
- `profile_3d_aop.cfg` contains `clutterRemoval -1 0`, `frameCfg ... 100 ...` (100 ms period), and `guiMonitor -1 1 1 1 0 0 1`. The original file is preserved.

## Architecture and files

- `radar_processing.py`: validation, ROI, motion/direction classes, connected-component clustering, median representatives, nearest-centroid tracks, range-rate, persistence, confidence, TTC, risk, target selection, and robot state. The shared `RadarProcessor.process_frame()` accepts live and replay frames.
- `radar_gui.py`: new object view, counters, map layers, settings, robot toggle, live serial connection, CSV replay, object CSV. It reuses the existing GUI module's `RadarWorker` and parser.
- `radar_gui_v1_ready.py`: adds a `raw_only` worker mode. The original point GUI mode remains intact. In raw mode, the existing CSV columns remain compatible; point `risk` is `N/A` and object calculations happen on the Tk thread.
- `profile_3d_aop_robot_clutter_on.cfg`: experimental copy with only clutter removal changed to `-1 1`.
- `test_radar_processing.py`: software tests.

## Pipeline

`Raw → finite/shape validation → ROI (range, front FOV, SNR) → STATIC/MOVING via Doppler deadband → APPROACHING/RECEDING via signed Doppler → x/y connected components → median object → greedy nearest-centroid match → range-rate from distance/time history → persistence and confidence → confirmed TTC → risk → lowest-TTC target`.

The defaults are: range 0.15–0.90 m, SNR ≥10 dB, FOV ±60°, Doppler deadband 0.10 m/s, approach sign -1, cluster distance 0.20 m, minimum 2 points, track match 0.30 m, maximum matched radial speed 2.0 m/s, distance smoothing alpha 0.35, temporal window 3 frames with 3 required, release after 3 misses, settle 0.30 s, WATCH/WARNING/AVOID/STOP TTC 3.0/2.0/1.2/0.6 s, emergency distance 0.20 m. All are editable with Apply; applying resets live tracks or rebuilds replay history to the current position.

Representative centroid, raw distance, and signed Doppler use medians. Matched object distance is then smoothed with an exponential update (`alpha=0.35`); the raw median stays in `raw_distance` for diagnosis. An association implying radial speed above 2.0 m/s starts a new tentative track. Range-rate is the median of up to three valid consecutive `(previous smoothed distance - current smoothed distance) / dt` estimates. HIGH confidence requires two consecutive positive rates and confirmation. MEDIUM can use confirmed Doppler with missing positive trend. A confirmed stationary or receding range trend stays LOW. TTC and target selection now require HIGH confidence; the source field remains explicit. Unconfirmed, LOW/MEDIUM, and paused objects have no TTC/risk target. A missing confirmed target is held as `LOST_TEMPORARY` for up to two frames with TTC and risk `N/A`, then released on the third miss.

## Robot state and future interfaces

`RobotStateManager.set_moving(bool, timestamp)` is the future ESP32 telemetry entry point. `MOVING` keeps UART input and raw logging active, while processing is PAUSED and target is None. Stopping enters SETTLING, then MONITORING after the configured interval. Camera fusion can consume confirmed object records after `process_frame()` and match them with camera detections; no camera path exists yet.

## Replay and logging

Select an existing `radar_gui_log_*.csv` in the new GUI, then PLAY/PAUSE/STOP at 0.5x, 1x, 2x, or MAX. Frames group by `frame_id` and use original `timestamp_s`. The same `process_frame()` handles serial and replay input. Parameter Apply rebuilds replay results up to the current frame. Existing CSV cannot reconstruct empty frames because they had no rows; capture a future frame-level log if missed-frame analysis is needed.

Live mode writes legacy-compatible `radar_gui_log_*.csv` raw point rows and `radar_objects_*.csv` object rows beside the selected CFG. The object CSV includes timestamp, frame, robot state, ID, confirmation/persistence/confidence, states, count, centroid, distance, angle, both velocities, TTC/source, risk, and target selection. CSV replay does not write a new object file.

## Hardware comparison procedure

Use the same physical setup, range, orientation, GUI parameters, and timed scenarios for both CFGs. Select the original CFG for OFF, run a stationary scene, slow/fast hand approach, hand receding, fixed object, MOVING toggle, and STOP/SETTLING. Save both raw and object CSVs. Repeat with the experimental clutter ON CFG. Compare raw/ROI/static/moving/approaching/cluster/confirmed counts, false target events in static scenes, and slow-moving target misses. Neither CFG is yet selected as the final setting. Verify Doppler sign, angle sign, distance accuracy, TTC, and latency against observed motion before connecting any avoidance command.

## Verification and limitations

`python -m unittest -v test_radar_processing.py` and `python -m py_compile radar_processing.py radar_gui.py radar_gui_v1_ready.py` passed. The Tk window can be constructed. The supplied CSV loads as 363 frames and 26,537 point rows through the shared processor. This is **SOFTWARE_VERIFIED** only. Serial hardware, radar RF behavior, physical motion, clutter comparison, GUI visual layout on the target screen, and false-positive/miss rates remain **HARDWARE_TEST_REQUIRED**. No **HARDWARE_VERIFIED** claim is made.
