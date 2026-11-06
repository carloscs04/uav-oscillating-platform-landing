# ground_station/receivers/nodes/control_rx.py

import json
import socket
import threading
import time
from typing import List, Tuple


class ControlReceiver(threading.Thread):
    """Receives 3-axis target setpoints (pos_ref) and actual position feedback (pos_act)."""

    def __init__(self, host: str = "0.0.0.0", port: int = 5007):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.running = False

        self._lock = threading.Lock()
        self.pos_ref: List[float] = [0.0, 0.0, 1.5]
        self.pos_act: List[float] = [0.0, 0.0, 0.0]
        self.last_seen: float = 0.0

    def run(self):
        """Background thread listening for incoming UDP telemetry packets."""
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
                        self.pos_ref = payload.get("ref", self.pos_ref)
                        self.pos_act = payload.get("act", self.pos_act)
                        self.last_seen = time.time()
            except socket.timeout:
                continue
            except Exception:
                time.sleep(0.05)

        sock.close()

    def get_control_data(self) -> Tuple[List[float], List[float]]:
        """Thread-safe retrieval of reference setpoints and actual position telemetry."""
        with self._lock:
            return list(self.pos_ref), list(self.pos_act)

    def stop(self):
        """Safely stops the thread loop."""
        self.running = False