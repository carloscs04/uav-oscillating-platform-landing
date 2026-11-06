# ground_station/receivers/nodes/distance_sensor_rx.py

import json
import socket
import threading
import time


class DistanceSensorReceiver(threading.Thread):
    """Receives rangefinder and distance sensor telemetry packets (Port 5008)."""

    def __init__(self, host: str = "0.0.0.0", port: int = 5008):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.running = False

        self._lock = threading.Lock()
        self.distance: float = 0.0
        self.last_seen: float = 0.0

    def run(self):
        """Background UDP listener loop for distance measurements."""
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
                data, _ = sock.recvfrom(1024)
                if data:
                    payload = json.loads(data.decode("utf-8"))
                    with self._lock:
                        self.distance = float(payload.get("distance", 0.0))
                        self.last_seen = time.time()
            except socket.timeout:
                continue
            except Exception:
                time.sleep(0.05)

        sock.close()

    def get_distance(self) -> float:
        """Thread-safe getter for the latest rangefinder distance."""
        with self._lock:
            return self.distance

    def stop(self):
        """Safely stops the thread loop."""
        self.running = False