class RobotMotionProvider:
    """Future ESP32 telemetry adapter; this phase never sends motor commands."""
    def is_moving(self):
        return False


class ManualMotionProvider(RobotMotionProvider):
    def __init__(self, moving=False):
        self.moving = bool(moving)

    def is_moving(self):
        return self.moving
