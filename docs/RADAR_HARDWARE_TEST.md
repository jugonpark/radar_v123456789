# Radar subsystem hardware characterization

This tool measures the existing radar subsystem independently. It imports the
working `RadarSerialSource`, Windows `parse_frame`, `RadarProcessor`, and Pi
profiles. It does not send UDP, motor commands, or change profiles, CFAR, firmware,
chirps, thresholds, or Doppler sign. No grise repository changes are required.

## Raspberry Pi: exact execution order

Close any existing radar runtime/GUI before opening the serial ports.

```bash
git clone https://github.com/jugonpark/radar_v123456789.git
cd radar_v123456789
git checkout feature/raspberry-pi-radar
git pull --ff-only
sudo apt update
sudo apt install -y python3-venv python3-tk usbutils
sudo usermod -aG dialout "$USER"
# Log out and back in after adding dialout membership.
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-pi.txt
python -m serial.tools.list_ports -v
python tools/pi_radar_hardware_check.py --cli-port /dev/ttyUSB0 --data-port /dev/ttyUSB1 --duration 30 --verbose --diag-log
python tools/pi_radar_hardware_check.py --cli-port /dev/ttyUSB0 --data-port /dev/ttyUSB1 --profile BALANCED --interactive --duration 10 --raw-log --diag-log
```

For an existing clone, begin at `cd`, checkout, and pull. Enhanced `/dev/ttyUSB0`
is CLI at 115200; Standard `/dev/ttyUSB1` is DATA at 921600. Omitting port arguments
uses existing discovery: udev symlinks, CP2105 metadata, then manual fallback.
The same CP2105 board should expose two **different serial interfaces**; the two
ports are not required to be two physical boards. Open success and role-label
verification are separate: unavailable labels produce WARN and explicitly record
manual/fallback mapping, rather than guessed PASS.

`python3-tk` is required because the existing parser lives in the Windows GUI
module; the checker itself creates no window and works without DISPLAY.

## Windows CMD

```cmd
cd /d "C:\Users\jugon\Documents\카카오톡 받은 파일\radar"
python tools\pi_radar_hardware_check.py --cli-port COM5 --data-port COM3 --duration 30 --verbose --diag-log
python tools\pi_radar_hardware_check.py --cli-port COM5 --data-port COM3 --interactive --duration 10 --raw-log --diag-log
```

Verify COM mapping with `python -m serial.tools.list_ports -v`; COM numbers can change.
Ctrl+C saves partial observations and records an interruption. A Windows run
does not validate Raspberry Pi CPU/USB behavior.

## Scenarios and interpretation

Each scenario waits for Enter, counts 3/2/1, then records for `--duration` seconds.
Queues are discarded before capture and tracking is reset between scenarios.
No motion ground truth is assumed.

| Scenario | User action | Main observation |
|---|---|---|
| EMPTY | Keep everything still | Baseline moving points/clusters and targets |
| STATIC_HAND | Hold hand at roughly 0.5–1.0 m | Raw reflection/SNR, not target requirement |
| SLOW_APPROACH | Slowly move 1.2 m toward 0.3 m | Raw distance trend, Doppler, candidate/range rate |
| FAST_APPROACH | Quickly move 1.2 m toward 0.3 m | Direction, track IDs, confirmed/high/target survival |
| RECEDE | Move 0.3 m away toward 1.2 m | Increasing distance, opposite Doppler, false approach |
| SIDE_MOTION | Move sideways at similar range | Radial approach counts and false targets |

Hold the ending position until the next instruction; do not return the hand
within an approach capture. Repeat complete runs for reliability evidence.
All raw points enter characterization, including points rejected by processing.
Distance trend is linear regression of per-frame **whole-cloud median range**;
it can be dominated by static reflectors. Moving Doppler median uses raw values
outside the existing profile's deadband. It is only supplemental sign evidence,
not identified-hand tracking. Scene results also include range-rate distribution,
target/object frame fractions, distinct track IDs, and stage counters.

## Criteria

- Port PASS: distinct interfaces successfully opened and Enhanced/Standard labels
  verified. Missing labels => WARN. Open failure => FAIL.
- CLI PASS: existing CFG completed and every command reported Done. Prompt-only
  acknowledgement => WARN. Error/timeout reports the command and stops capture.
  Data-open failure does not erase successful CLI configuration evidence.
- DATA FAIL: no parsed frames. Parse/length/point-count/TLV-envelope problems =>
  WARN; otherwise PASS. `valid_packets` means framing-length-valid packets, not
  proven-valid RF measurements. Resyncs/total bytes are recorded separately.
- Frame stability PASS: no gaps, duplicates or out-of-order frames and measured
  FPS falls within `--fps-tolerance` (default 20%) of expected FPS. Expected FPS
  comes from CFG `frameCfg` period or `--expected-fps`; no fixed 8–12 Hz rule.
- Sign PASS: FAST_APPROACH whole-cloud range slope decreases, RECEDE increases,
  and moving Doppler agrees with the current approach sign. Supplemental
  evidence floors are |Doppler| >= 0.1 m/s and |range slope| >= 0.02 m/s.
  Insufficient evidence => UNKNOWN. Disagreement => WARN; sign is never changed.
- RAW response PASS: raw geometry exists and paired scenario sign/trend evidence
  agrees. Raw existence alone => WARN, no valid geometry => FAIL. This is scene
  evidence and is not proof that each reflection belongs to the hand.
- Processing PASS/READY: all six scenarios exist, sign evidence passes, both
  approach scenarios produce a target, and the four other scenarios produce no
  target. Otherwise WARN/TUNING_REQUIRED. These are observed scenario criteria,
  not authorization for integration or actuation.
- Target reliability remains WARN pending repeated labelled trials. One run
  cannot establish recall or false-positive rate. Empty raw reflections and a
  static hand with no target are not sensor failures.
- SENSOR READY requires successful transport/configuration, stable frames,
  sign evidence and raw geometry. Missing scenarios/evidence => NEEDS_RETEST.
  Transport/configuration failure => HARDWARE_PROBLEM (investigate cable,
  permissions and configuration; it does not necessarily mean damaged hardware).
  Interruption => NEEDS_RETEST. Target absence alone never makes SENSOR FAIL.

Stage tables show totals, mean/frame and ratio to raw point count. Point-stage
ratios are point retention; cluster/object/track/target ratios have different
units and **must not** be read as point survival probabilities. Rejection counts
use existing mutually exclusive RANGE → FOV → SNR precedence. Largest rejection
is computed from the new run; zero rejections => NONE.

The working parser's payload-only TLV-length convention and unsigned side-info
are unchanged. Envelope bounds and advertised/decoded point-count mismatches
are diagnostic counters. They do not establish firmware-format correctness or
replace packet inspection on actual hardware.

## Saved artifacts

Each invocation creates `logs/hardware_test/YYYYMMDD_HHMMSS_microseconds/`:

- `objects.csv`: scenario, frame, receive monotonic time and full object JSON.
- `scenarios.csv`: labelled scenario capture boundaries and counts.
- `raw_points.csv`: optional `--raw-log`; XYZ/range/angle/Doppler/SNR/noise.
- `frame_diagnostics.csv`: optional `--diag-log`; per-frame processing stages.
- `summary.json`: port metadata, CLI commands/responses, transport counters,
  continuity, distributions/percentiles, stage statistics, scenarios and verdicts.
- `test_report.txt`: readable verdicts plus full summary.

Raw logging defaults OFF. Tests are duration bounded; scenarios require six times
that capture duration plus preparation time. Delete/archive test directories as
needed on the SD card. `--output-dir` changes the parent directory. Date labels
use wall time; intervals, deadlines and latency use monotonic time.

Latency statistics measure host parser and RadarProcessor execution, not RF
capture-to-host absolute latency, UART transfer time, or CSV serialization.
Host inter-frame intervals are delivery intervals and include host scheduling.
Parser samples retain the latest 10,000 packets; processor statistics cover the
test captures. Do not use these numbers as RF timestamp measurements.

## Optional UDP pre-integration probe

Stop the checker before starting the ordinary runtime. Laptop:

```bash
python tools/pi_radar_udp_probe.py --port 8890 --stale-seconds 1
```

Pi, replacing the address with the laptop's actual Wi-Fi IPv4 address:

```bash
python raspberry_pi/pi_radar_main.py --telemetry-host 192.168.1.20 --telemetry-port 8890
```

Allow UDP 8890 in the laptop firewall and synchronize laptop/Pi clocks. The probe
prints packet rate, malformed count, health, frame, receive age, sender wall age
and target (distance/angle/range-rate/TTC/risk). Local monotonic silence detects
missing traffic. Old/future sender timestamps, stale/invalid health and malformed
schemas suppress usable targets. Sender wall age is meaningful only with aligned
clocks; UDP has no authentication and is only a local characterization tool.
Malformed packets do not refresh validity. This does not connect to grise.

## USB reconnect: HARDWARE_TEST_REQUIRED

Automatic reconnect scenario is deliberately not advertised by this checker.
Run the Pi runtime separately; unplug/replug USB and record DISCONNECTED,
rediscovery, CLI/configuration resend, DATA frame recovery, fresh tracking and
SETTLING → MONITORING. This sequence still needs actual Pi hardware observation;
mock tests do not prove USB recovery. The checker records exceptions/partial
results and ends rather than silently mixing captures across reconnects.

## Software verification

```bash
python -m unittest discover -v
python -m compileall -q raspberry_pi tools test_pi_hardware_check.py
python tools/pi_radar_hardware_check.py --help
python tools/pi_radar_udp_probe.py --help
```

Unit/regression/import results are SOFTWARE_VERIFIED. Actual Pi transport, RF
response, Doppler sign, hand detection, SD-card/CPU behavior, Wi-Fi and USB
reconnect remain HARDWARE_TEST_REQUIRED until the operator runs these procedures.


## V2 track diagnostics and offline replay

See [Track Fast Path V2](RADAR_FAST_PATH_V2.md) for Windows/Pi commands, recorded ACTION_START, full/action-window comparisons and offline threshold tables. Use `--action-window 2 --raw-log --diag-log` with the interactive checker. Thresholds are EXPERIMENTAL and HARDWARE_TUNING_REQUIRED; OBSERVE retains baseline targets. Replay does not validate sensor hardware.
