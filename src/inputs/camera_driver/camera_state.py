# File: src/inputs/camera_driver/camera_state.py

import threading
from typing import Any, Dict, Optional


class CameraState:
    """Thread-safe buffer dedicated exclusively to Camera video frames and stream status."""

    def __init__(self):
        self._lock = threading.Lock()
        self.latest_b64_frame: str = ""
        self.resolution: str = "640x480"
        self.fps_actual: float = 30.0

    def update_frame(self, b64_data: str, fps: Optional[float] = None, resolution: Optional[str] = None):
        """Update the latest Base64-encoded video frame and optional stream metrics."""
        with self._lock:
            self.latest_b64_frame = b64_data
            if fps is not None:
                self.fps_actual = round(fps, 1)
            if resolution is not None:
                self.resolution = resolution

    def export_telemetry_data(self) -> Dict[str, Any]:
        """Export video frame and camera stream metadata packets."""
        with self._lock:
            return {
                "video_frame": {
                    "data": self.latest_b64_frame
                },
                "camera_status": {
                    "resolution": self.resolution,
                    "fps": self.fps_actual
                }
            }

    def export_node_health(self, rate_hz: float, is_connected: bool) -> Dict[str, Any]:
        """Export health status for the Camera stream node."""
        if not is_connected:
            return {
                "status": "RECONNECTING",
                "rate_hz": 0.0,
                "battery_pct": -1,
                "voltage_v": 0.0
            }

        health_status = "HEALTHY" if rate_hz >= 10.0 else ("POOR" if rate_hz > 0.0 else "RECONNECTING")
        return {
            "status": health_status,
            "rate_hz": round(rate_hz, 1),
            "battery_pct": -1,
            "voltage_v": 0.0
        }