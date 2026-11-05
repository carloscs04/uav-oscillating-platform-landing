import os
import time
import subprocess
import threading
from ground_station.utils.yaml_loader import get_profile_data, load_yaml

class LinkState:
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    RECONNECTING = "RECONNECTING"
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    STOPPED = "STOPPED"

class PiLink:
    def __init__(self, config_dir: str, mode: str = "school"):
        self.config_dir = config_dir
        self.requested_mode = mode
        
        # Load timeout settings
        settings_path = os.path.join(config_dir, "settings.yaml")
        settings = load_yaml(settings_path).get("network", {})
        self.ping_timeout = settings.get("ping_timeout_sec", 1)
        
        # State Management
        self.state = LinkState.DISCONNECTED
        self.active_mode = mode
        self.active_ip = "N/A"
        self.latency_ms = -1.0
        self.consecutive_failures = 0
        self.max_failures_before_reconnect = 2
        
        # Internal Thread Control
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        
        # Profile Initialization
        self._apply_profile(self.requested_mode)

    def _apply_profile(self, mode: str):
        """Loads connection settings for selected profile."""
        profile_info = get_profile_data(self.config_dir, mode)
        with self._lock:
            self.active_mode = profile_info["mode"]
            self.active_ip = profile_info["pi_ip"]
            self.port_telemetry = profile_info["port_telemetry"]
            self.port_command = profile_info["port_command"]
            self.port_video = profile_info["port_video"]
            self.fallback_sequence = profile_info.get("fallback_sequence", ["wifi", "usb", "ap"])

    def start(self):
        """Starts the background monitoring and auto-reconnect engine."""
        if self._running:
            return
        
        self._running = True
        self.state = LinkState.CONNECTING
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Safely terminates the link engine."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        with self._lock:
            self.state = LinkState.STOPPED

    def switch_mode(self, new_mode: str):
        """Manual trigger to switch modes (e.g., wifi -> usb -> ap)."""
        with self._lock:
            self.requested_mode = new_mode
            self.consecutive_failures = 0
            self.state = LinkState.CONNECTING
        self._apply_profile(new_mode)

    def get_health(self) -> dict:
        """Returns thread-safe health metrics for the Master Panel UI."""
        with self._lock:
            return {
                "state": self.state,
                "mode": self.active_mode,
                "ip": self.active_ip,
                "latency_ms": round(self.latency_ms, 1) if self.latency_ms >= 0 else "N/A",
                "failures": self.consecutive_failures,
                "ports": {
                    "telemetry": self.port_telemetry,
                    "command": self.port_command,
                    "video": self.port_video
                }
            }

    def _ping(self, ip: str) -> tuple[bool, float]:
        """Pings the target IP and measures latency in milliseconds."""
        if ip == "N/A":
            return False, -1.0
            
        is_win = os.name == "nt"
        cmd = ["ping", "-n", "1", "-w", str(int(self.ping_timeout * 1000)), ip] if is_win \
         else ["ping", "-c", "1", "-W", str(int(self.ping_timeout)), ip]
        
        start_time = time.time()
        try:
            res = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            latency = (time.time() - start_time) * 1000.0
            return (res.returncode == 0), latency
        except Exception:
            return False, -1.0

    def _worker_loop(self):
        """Continuous background execution loop handling pinging & dynamic reconnects."""
        while self._running:
            if self.active_mode == "auto":
                self._handle_auto_discovery()
            else:
                self._handle_direct_connection()
            
            time.sleep(1.0)

    def _handle_direct_connection(self):
        """Handles single-target connection check and seamless recovery."""
        success, latency = self._ping(self.active_ip)
        
        with self._lock:
            if success:
                self.consecutive_failures = 0
                self.latency_ms = latency
                self.state = LinkState.DEGRADED if latency > 150 else LinkState.HEALTHY
            else:
                self.consecutive_failures += 1
                self.latency_ms = -1.0
                
                if self.consecutive_failures >= self.max_failures_before_reconnect:
                    self.state = LinkState.RECONNECTING
                else:
                    self.state = LinkState.DEGRADED

    def _handle_auto_discovery(self):
        """Sequentially attempts fallback modes until an active Pi IP responds."""
        for fallback_mode in self.fallback_sequence:
            profile_info = get_profile_data(self.config_dir, fallback_mode)
            target_ip = profile_info.get("pi_ip")
            
            success, latency = self._ping(target_ip)
            if success:
                with self._lock:
                    self.active_mode = f"auto ({fallback_mode})"
                    self.active_ip = target_ip
                    self.latency_ms = latency
                    self.consecutive_failures = 0
                    self.state = LinkState.HEALTHY
                return

        with self._lock:
            self.consecutive_failures += 1
            self.latency_ms = -1.0
            self.state = LinkState.RECONNECTING