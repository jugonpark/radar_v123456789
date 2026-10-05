"""Optional Pi GUI entry point; use the existing GUI and processing path."""
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from radar_gui import RadarApp
from raspberry_pi.profiles import settings_for


class PiRadarApp(RadarApp):
    def _process_frame(self, fid, points, ts):
        return super()._process_frame(fid, points, time.monotonic() if self.mode == "LIVE" else ts)

    def _set_moving(self, moving):
        now = time.monotonic() if self.mode == "LIVE" else (self.last_result["timestamp"] if self.last_result else 0.0)
        self.processor.set_moving(moving, now)
        self._render(self.processor.process_frame(self.last_result["frame_id"] + 1 if self.last_result else 0, [], now))


def main():
    if not os.environ.get("DISPLAY") and sys.platform.startswith("linux"):
        print("GUI display unavailable. Run: python raspberry_pi/pi_radar_main.py")
        return 2
    app = PiRadarApp()
    app.processor.settings = settings_for("BALANCED")
    for name, var in app.settings_vars.items():
        var.set(str(getattr(app.processor.settings, name)))
    app.cli_var.set("/dev/ttyUSB0")
    app.data_var.set("/dev/ttyUSB1")
    app.show_tentative_var.set(True)
    app.layer_vars["Clusters"].set(True)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
