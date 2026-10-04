# GRISE radar + video integration

## Repository fit

- Active [grise](https://github.com/32200362-sys/grise/blob/master/main.py) currently runs camera perception, risk, potential-field planning and UDP sending in one PC process. Its latest `DANGER` status is sent as `RUN` so an avoidance velocity can reach the ESP32. `STOP` zeros that velocity.
- [smart_coaster_ai](https://github.com/jugonpark/smart_coaster_ai) is a three-node reference: laptop video, Pi perception/fusion/planning/safety, ESP32 UDP control. It has a placeholder radar path at UDP 8890. Do not substitute its unfinished serial parser for the working IWR6843 parser here.
- Desired Pi layout: camera stream plus IWR6843 USB serial input on Pi; Pi owns fused decision and sends the existing `{seq,t,vx,vy,w,status}` command to ESP32 UDP 8888. ESP32 retains watchdog, kinematics and motor control. If radar is processed in the same Pi process, UDP 8890 is unnecessary.

## Implemented here

`radar_processing.py` supplies confirmed targets from 0.15–0.90 m with clustering, tracking, smoothing, stationary-only range-rate and TTC. `grise_radar_gate.py` adds a 0.35 m/s fast-approach trigger, TTC <= 1.2 s, freshness limits and video-verified escape-sector requirement. It returns `REQUEST_AVOID`, `MONITOR`, or `STOP` and never sends a motor command.

## Wiring into the active GRISE checkout

1. Port `radar_processing.py`, `grise_radar_gate.py`, and the TLV serial parser from `radar_gui_v1_ready.py` into a Pi-side perception package. Feed each parsed `{frame, points}` into `RadarProcessor.process_frame(frame, points, time.monotonic())`. Use the same monotonic clock for video and gate timestamps. The parser should be the only reader of the radar DATA UART.
2. Build `VideoEvidence` from the existing calibrated camera frame: robot pose valid, calibration valid, and individually checked left/right/rear clearance. Unknown sectors must be `False`; a visible obstacle or hand in a sector must make it `False`. Preserve the existing hand/cup risk as another input to the overall safety decision.
3. Call `decide(result, video, now, robot_stopped)` before GRISE's planner/UDP sender. `MONITOR` leaves the existing GRISE behavior intact. `STOP` forces zero velocity and `status="STOP"`. `REQUEST_AVOID` is only permission to enter a bounded, video-checked avoidance routine; it is not a velocity command. Do not map radar angle directly to motor direction.
4. During avoidance the robot moves, so `RadarProcessor.set_moving(True, now)` pauses stationary range-rate and clears tracks. Bound the movement by time and distance using fresh camera pose, then STOP, call `set_moving(False, now)`, wait through settling and collect new radar frames. If video, pose, calibration, UDP telemetry or command freshness is lost at any point, STOP. Keep automatic escape motion disabled until physical tests establish mount yaw, Doppler sign, latency and safe sector geometry.
5. Preserve the active GRISE `UdpSender` and ESP32 status whitelist. `REQUEST_AVOID` must pass through GRISE's safety/planning limiters before sending `RUN`; never send `DANGER` as the UDP status string.

The current working directory is the radar project, not a checkout of `32200362-sys/grise`; this document is a concrete porting guide, not a claim that the GitHub repository was changed. Software tests are available with `python -m unittest test_grise_radar_gate test_radar_processing -v`. Physical radar, video alignment, and robot motion remain unverified.
