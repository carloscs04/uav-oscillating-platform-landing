# ground_station/receivers/nodes/camera_rx.py

import socket
import threading
import time


class CameraReceiver(threading.Thread):
    """Receives camera video stream packets (Port 8080)."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8080):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.running = False

        self._lock = threading.Lock()
        self.last_frame = None
        self.last_seen: float = 0.0

    def run(self):
        """Background UDP listener loop for camera frame data."""
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
                data, _ = sock.recvfrom(65535)
                if data:
                    with self._lock:
                        self.last_frame = data
                        self.last_seen = time.time()
            except socket.timeout:
                continue
            except Exception:
                time.sleep(0.05)

        sock.close()

    def get_latest_frame(self):
        """Thread-safe getter for the most recent video frame buffer."""
        with self._lock:
            return self.last_frame

    def stop(self):
        """Safely stops the thread loop."""
        self.running = False