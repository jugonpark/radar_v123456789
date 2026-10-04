"""Optional Pi GUI entry point; use the existing GUI and processing path."""
import os
import sys

from radar_gui import RadarApp


def main():
    if not os.environ.get("DISPLAY") and sys.platform.startswith("linux"):
        print("GUI display unavailable. Run: python raspberry_pi/pi_radar_main.py")
        return 2
    RadarApp().mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
