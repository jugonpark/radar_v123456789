# Raspberry Pi radar runtime design

## Goal

Run the existing IWR6843 parser and `RadarProcessor` on Raspberry Pi 5 while preserving the Windows GUI and replay flow. The Pi owns radar parsing, tracking, TTC and risk; a GUI and laptop telemetry are optional consumers. No ESP32 motor command is sent in this phase.

## Boundaries

`radar_gui_v1_ready.parse_frame` remains the packet parser. `radar_processing.RadarProcessor` remains the only processing implementation. Pi code supplies serial lifecycle, CLI response validation, health/fail-safe state, object logging, telemetry and optional UI.

Port selection is stable symlinks first, CP2105 VID/PID/interface second, and `/dev/ttyUSB0`/`/dev/ttyUSB1` last. CLI is 115200 and DATA is 921600. All elapsed-time decisions use `time.monotonic()`; wall time is used only in CSV filenames and display timestamps.

## Runtime flow

`RadarRuntime` opens CLI, sends `sensorStop` and CFG lines, validates response text, then opens DATA. Complete packets are parsed by the existing parser and passed to `RadarProcessor.process_frame`. A DATA timeout sets `STALE` and clears the target. Serial errors set `DISCONNECTED`; a reconnect attempt closes both ports, rediscoveries ports, reapplies CFG, resets tracks and enters `SETTLING` before monitoring.

The runtime writes object CSV by default and raw point CSV only when enabled. Object logs rotate by size. Telemetry is JSON over UDP at no more than 10 Hz and contains counters and one target, never raw points.

## Safety states

`CONNECTING`, `CONFIGURING`, `OK`, `STALE`, `DISCONNECTED`, `CONFIG_ERROR`, and `ERROR` are explicit. `STALE`, `DISCONNECTED`, `CONFIG_ERROR`, and `ERROR` expose `target=None`, `risk=N/A`, and `radar_valid=false`. A motion provider interface is included; its default is stopped/manual and no motor sender exists.

## Verification boundary

Unit and mock-serial tests verify software behavior. Actual CLI acknowledgements, CP2105 interface names, udev rules, USB reconnect, RF/TLV behavior, Pi resource usage and robot motion require hardware testing.

## Detection sensitivity revision

Pi `BALANCED` uses 1.5 m detection and 0.9 m threat distance. It clusters moving points before classifying object direction so one mixed-sign point does not erase a small object. One-point clusters become tentative candidates; 2 of 3 detections confirm a track, while threat selection still requires approaching direction, high confidence, positive range trend and a risk within the threat range. Pi `DIAGNOSTIC` admits missing SNR solely for observation and suppresses target selection. `STRICT` retains Windows defaults and its prior point-direction clustering order. Per-frame reject counters and optional diagnostics CSV show where detections disappear. These parameters require labeled hand tests on the Pi before they can be considered calibrated.
