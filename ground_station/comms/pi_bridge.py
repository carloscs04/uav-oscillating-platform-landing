# ground_station/comms/pi_bridge.py

import os
import time
import subprocess
import threading
from ground_station.utils.yaml_loader import get_profile_data, load_yaml


class LinkState:
    """Raw transport / connection status."""
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    RECONNECTING = "RECONNECTING"
    CONNECTED = "CONNECTED"
    STOPPED = "STOPPED"


class HealthState:
    """Metric-driven connection quality."""
    HEALTHY = "HEALTHY"    # Low latency (<= 150ms)
    DEGRADED = "DEGRADED"  # High latency (> 150ms)
    OFFLINE = "OFFLINE"    # No heartbeat / disconnected


class PiLink:
    """Low-level network engine tracking connection state and quality metrics independently."""

    LATENCY_THRESHOLD_MS = 150.0
    _active_instance = None  # Holds global reference to the live GUI instance

    def __init__(self, config_dir: str, mode: str = "wifi"):
        PiLink._active_instance = self  # Register this live instance globally

        self.config_dir = config_dir
        self.requested_mode = mode

        # Load timeout settings
        settings_path = os.path.join(config_dir, "settings.yaml")
        settings = load_yaml(settings_path).get("network", {})
        self.ping_timeout = settings.get("ping_timeout_sec", 1)

        # Connection and Health states
        self.link_state = LinkState.DISCONNECTED
        self.health_state = HealthState.OFFLINE

        # Flag tracking if connection was achieved at least once during session
        self.has_ever_connected = False

        self.active_mode = mode
        self.active_ip = "N/A"
        self.latency_ms = -1.0
        self.consecutive_failures = 0
        self.max_failures_before_reconnect = 2

        self.port_telemetry = 5005
        self.port_command = 5006
        self.port_video = 8080

        self.running = False
        self._thread = None
        self._lock = threading.Lock()

        # Load initial profile parameters directly
        self.load_profile(mode)

    def start(self):
        """Starts background monitoring worker thread."""
        with self._lock:
            if self.running:
                return
            self.running = True
            self.link_state = LinkState.CONNECTING
            self.health_state = HealthState.OFFLINE
            self._thread = threading.Thread(target=self._worker_loop, daemon=True)
            self._thread.start()

    def stop(self):
        """Stops background worker thread instantly without blocking the main GUI thread."""
        self.running = False
        with self._lock:
            self.link_state = LinkState.STOPPED
            self.health_state = HealthState.OFFLINE

    def load_profile(self, new_mode: str):
        """Loads profile configuration directly from YAML and updates network targets without helper functions."""
        with self._lock:
            self.requested_mode = new_mode
            self.consecutive_failures = 0

            # Load profile configuration directly
            prof = get_profile_data(self.config_dir, new_mode)
            self.active_mode = prof.get("mode", new_mode)
            self.active_ip = prof.get("pi_ip", "10.22.246.2")
            self.port_telemetry = prof.get("port_telemetry", 5005)
            self.port_command = prof.get("port_command", 5006)
            self.port_video = prof.get("port_video", 8080)

            if not self.running:
                self.link_state = LinkState.DISCONNECTED
            self.health_state = HealthState.OFFLINE

    def _ping(self) -> float:
        """Measures ICMP ping latency in milliseconds."""
        if self.active_ip in ["N/A", "127.0.0.1", "localhost"]:
            return 5.0

        try:
            start_t = time.time()
            cmd = ["ping", "-c", "1", "-W", str(self.ping_timeout), self.active_ip]
            res = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            if res.returncode == 0:
                return (time.time() - start_t) * 1000.0
        except Exception:
            pass

        return -1.0

    def _worker_loop(self):
        """Background thread monitoring network quality."""
        while self.running:
            latency = self._ping()

            with self._lock:
                if not self.running:
                    break

                if latency >= 0:
                    self.latency_ms = latency
                    self.consecutive_failures = 0
                    self.has_ever_connected = True

                    self.link_state = LinkState.CONNECTED
                    if latency <= self.LATENCY_THRESHOLD_MS:
                        self.health_state = HealthState.HEALTHY
                    else:
                        self.health_state = HealthState.DEGRADED
                else:
                    self.latency_ms = -1.0
                    self.consecutive_failures += 1
                    self.health_state = HealthState.OFFLINE

                    if not self.has_ever_connected:
                        # 1. First run: Keep state as CONNECTING until first successful link
                        self.link_state = LinkState.CONNECTING
                    else:
                        # 2. Subsequent connection drop: Shift immediately to RECONNECTING
                        self.link_state = LinkState.RECONNECTING

            time.sleep(1.0)

    def get_health(self) -> dict:
        """Returns thread-safe connection and health state parameters."""
        with self._lock:
            return {
                "state": self.link_state,
                "health": self.health_state,
                "mode": self.active_mode,
                "ip": self.active_ip,
                "latency_ms": round(self.latency_ms, 1) if self.latency_ms >= 0 else "N/A",
                "failures": self.consecutive_failures,
                "has_ever_connected": self.has_ever_connected,
                "ports": {
                    "telemetry": self.port_telemetry,
                    "command": self.port_command,
                    "video": self.port_video,
                },
            }