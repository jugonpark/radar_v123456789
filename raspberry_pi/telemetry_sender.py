import json
import socket
import time


class TelemetrySender:
    def __init__(self, host, port, hz=10.0, enabled=True, sock=None):
        self.address = (host, int(port))
        self.interval = 1.0 / max(float(hz), 0.1)
        self.enabled = enabled
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_send = 0.0

    def send(self, payload, now=None, force=False):
        if not self.enabled:
            return False
        now = time.monotonic() if now is None else now
        if not force and now - self.last_send < self.interval:
            return False
        self.sock.sendto(json.dumps(payload, separators=(",", ":")).encode(), self.address)
        self.last_send = now
        return True

    def close(self):
        self.sock.close()
