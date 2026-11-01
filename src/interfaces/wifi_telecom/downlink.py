import json
import time
import socket
import threading
from .telemetry_hub import TelemetryHub


class DownlinkSender:
    """Self-contained telemetry transmitter."""

    def __init__(self, hub: TelemetryHub):
        self.hub = hub
        self.port = hub.config["telemetry_port"]
        self.rate_hz = hub.config["telemetry_rate_hz"]
        self.is_running = False

        self._sock = None
        self._thread = None
        self._lock = threading.Lock()
        self._telemetry_data = {}

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.is_running = True
        self._thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._thread.start()

    def push_telemetry(self, data_dict: dict):
        """Called by MasterNode to pass current flight telemetry."""
        with self._lock:
            self._telemetry_data = data_dict

    def _tx_loop(self):
        interval = 1.0 / self.rate_hz
        while self.is_running:
            target_ip = self.hub.get_pc_ip()

            if target_ip and self._sock:
                with self._lock:
                    payload = dict(self._telemetry_data)

                if payload:
                    try:
                        payload["timestamp"] = round(time.time(), 3)
                        bytes_data = json.dumps(payload).encode("utf-8")
                        self._sock.sendto(bytes_data, (target_ip, self.port))
                    except Exception:
                        pass  # Packet drops cause zero onboard disruption

            time.sleep(interval)

    def stop(self):
        self.is_running = False
        if self._sock:
            self._sock.close()