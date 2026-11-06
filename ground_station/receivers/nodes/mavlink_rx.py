# ground_station/receivers/nodes/mavlink_rx.py

import json
import socket
import threading
import time
from typing import Any, Dict


class MAVLinkReceiver(threading.Thread):
    """Receives PX4 / MAVLink flight telemetry packets (Port 5005)."""

    def __init__(self, host: str = "0.0.0.0", port: int = 5005):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.running = False

        self._lock = threading.Lock()
        self.telemetry: Dict[str, Any] = {}
        self.last_seen: float = 0.0

    def run(self):
        """Background UDP listener loop for MAVLink telemetry."""
        self.running = True
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(1.0)

        try:
            sock.bind((self.host, self.port))
        except Exception:
            pass

        while self.running:
            try:
                data, _ = sock.recvfrom(2048)
                if data:
                    payload = json.loads(data.decode("utf-8"))
                    with self._lock:
                        self.telemetry = payload
                        self.last_seen = time.time()
            except socket.timeout:
                continue
            except Exception:
                time.sleep(0.05)

        sock.close()

    def get_telemetry(self) -> Dict[str, Any]:
        """Thread-safe getter for current MAVLink telemetry state."""
        with self._lock:
            return dict(self.telemetry)

    def stop(self):
        """Safely stops the thread loop."""
        self.running = False