import os
import threading
from src.utils.config_loader import load_yaml_config


class TelemetryHub:
    """Central registry holding shared network state and active ground PC IP."""

    def __init__(self):
        config_path = os.path.join(os.path.dirname(__file__), "config.yaml")
        defaults = {"telemetry_port": 5005, "video_port": 8080, "telemetry_rate_hz": 20}
        self.config = load_yaml_config(config_path, defaults)

        self._lock = threading.Lock()
        self.active_pc_ip = None

    def register_pc_ip(self, ip_address: str):
        """Called by uplink.py when a command packet arrives from a PC."""
        with self._lock:
            if self.active_pc_ip != ip_address:
                self.active_pc_ip = ip_address
                print(f"[TelemetryHub] Active Ground PC locked at IP: {ip_address}")

    def get_pc_ip(self) -> str:
        """Called by downlink.py to query where to stream telemetry."""
        with self._lock:
            return self.active_pc_ip