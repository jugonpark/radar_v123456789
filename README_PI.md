# Raspberry Pi 5 radar runtime

This adds a Linux-only entry point while preserving the Windows GUI and replay files. The Pi reuses `radar_gui_v1_ready.parse_frame` and `radar_processing.RadarProcessor`; no second processing algorithm exists.

## Install

```bash
git clone https://github.com/jugonpark/radar_v123456789.git
cd radar_v123456789
sudo apt update
sudo apt install -y python3 python3-pip python3-venv python3-tk git usbutils
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-pi.txt
sudo usermod -aG dialout $USER
python -m serial.tools.list_ports -v
```

The verified mapping is `/dev/ttyUSB0` Enhanced/CLI/115200 and `/dev/ttyUSB1` Standard/DATA/921600. The runtime prefers `/dev/radar_cli` and `/dev/radar_data`, then CP2105 VID `10c4` PID `ea70`, then USB fallback.

## Run

```bash
python raspberry_pi/pi_radar_main.py
python raspberry_pi/pi_radar_main.py --no-auto-port --cli-port /dev/ttyUSB0 --data-port /dev/ttyUSB1 --no-telemetry
python raspberry_pi/pi_radar_gui.py
```

The default CFG is `profile_3d_aop.cfg`; pass `--cfg profile_3d_aop_robot_clutter_on.cfg` for the experimental profile. Object logs go to `./logs`; raw logging is off unless `--raw-log` is supplied. SSH sessions without `DISPLAY` should use headless mode.

## Hand detection diagnostics

The Pi starts with `--profile BALANCED`. `STRICT` retains the earlier 0.15–0.90 m, SNR 10 dB, two-point, 3/3 behavior. `BALANCED` detects and tracks from 0.15–1.50 m with SNR 8 dB, one-point candidates and 2/3 persistence; threat risk remains inside 0.90 m. A two-frame candidate reaches high confidence only when representative Doppler and measured distance trend both show approach. `DIAGNOSTIC` accepts missing SNR for observation but never selects a target. These are software tuning starting points, not verified hand-detection thresholds.

For a short labeled hand test, run:

```bash
python raspberry_pi/pi_radar_main.py --profile BALANCED --verbose --diag-log --raw-log
```

Record 10 seconds empty, then a stationary hand, slow approach, fast approach, one quick wave and retreat. `--diag-log` writes one row per frame with stage survival and reject counts. `--raw-log` writes all points and should only be used for short tests. The Pi GUI shows tentative objects by default; a single-point candidate is a small marker and never immediately becomes a threat. The headless summary shows moving points, clusters, tentative objects and confirmed objects so a missing target can be traced to its stage. Compare profiles with `--profile STRICT` or `--profile DIAGNOSTIC` using the same motion sequence.

## Ports, udev and service

```bash
lsusb
python -m serial.tools.list_ports -v
udevadm info -a -n /dev/ttyUSB0
sudo cp udev/99-iwr6843.rules.example /etc/udev/rules.d/99-iwr6843.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/radar_cli /dev/radar_data
```

Verify attributes before installing the example rule. For systemd, replace `<USER>` in `systemd/radar-pi.service.example`, copy it to `/etc/systemd/system/`, then explicitly run `sudo systemctl daemon-reload` and `sudo systemctl enable --now radar-pi.service`.

## Runtime behavior

Health states are `CONNECTING`, `CONFIGURING`, `OK`, `STALE`, `DISCONNECTED`, `CONFIG_ERROR`, and `ERROR`. Bad states clear target/risk and set `radar_valid=false`. Telemetry defaults to `127.0.0.1:8890`, maximum 10Hz, and contains no raw points. Processing and timeout calculations use `time.monotonic()`; wall time is used only for display/log timestamps.

Run `bash scripts/setup_pi.sh` for the guided apt, dialout, venv and dependency setup. Tkinter is installed by apt, not pip.

## Verification boundary

For isolated transport, raw-point characterization and interactive hand scenarios,
see [Hardware validation procedure](docs/RADAR_HARDWARE_TEST.md). Start with
`python tools/pi_radar_hardware_check.py --interactive --duration 10 --raw-log --diag-log`.
The checker sends no telemetry; an independent UDP receiver is available as
`python tools/pi_radar_udp_probe.py --port 8890`.

`SOFTWARE_VERIFIED`: unit tests, mock serial lifecycle, port selection, telemetry schema/rate limit, stale/moving fail-safe, import and syntax checks. `HARDWARE_TEST_REQUIRED`: actual IWR6843 CLI responses, CP2105 interface labels, udev symlinks, 921600 DATA stream, USB reconnect, CPU/RAM load and motor integration.


## V2 track diagnostics and offline replay

See [Track Fast Path V2](docs/RADAR_FAST_PATH_V2.md) for Windows/Pi commands, recorded ACTION_START, full/action-window comparisons and offline threshold tables. Use `--action-window 2 --raw-log --diag-log` with the interactive checker. Thresholds are EXPERIMENTAL and HARDWARE_TUNING_REQUIRED; OBSERVE retains baseline targets. Replay does not validate sensor hardware.


## Independent approach speed experiment

See [Approach Speed Decision](docs/RADAR_APPROACH_SPEED.md) for the exact Windows CMD/Pi four-scene `--approach-test`, distance-history speed thresholds and replay comparisons. Start with a strong metal reflector, then repeat with a hand. OBSERVE sends no motor commands and preserves legacy target/risk. SOFTWARE_VERIFIED and HARDWARE_TEST_REQUIRED remain separate.
