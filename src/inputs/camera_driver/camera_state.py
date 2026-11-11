# File: src/inputs/camera_driver/camera_state.py

import threading
from typing import Any, Dict, Optional


class CameraState:
    """Thread-safe buffer for Camera frames, status, and ArUco detection metadata."""

    def __init__(self):
        self._lock = threading.Lock()
        self.latest_b64_frame: str = ""
        self.resolution: str = "640x480"
        self.fps_actual: float = 30.0

        self.aruco_detected: bool = False
        self.aruco_id: int = -1
        self.aruco_pos: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0}

    def update_frame(self, b64_data: str):
        with self._lock:
            self.latest_b64_frame = b64_data

    def update_aruco_pose(self, detected: bool, target_id: int, x: float, y: float, z: float):
        with self._lock:
            self.aruco_detected = detected
            self.aruco_id = target_id
            self.aruco_pos = {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)}

    def export_telemetry_data(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "video_frame": {
                    "data": self.latest_b64_frame
                },
                "camera_status": {
                    "resolution": self.resolution,
                    "fps": self.fps_actual
                },
                "aruco_pose": {
                    "detected": self.aruco_detected,
                    "id": self.aruco_id,
                    "x": self.aruco_pos["x"],
                    "y": self.aruco_pos["y"],
                    "z": self.aruco_pos["z"]
                }
            }

    def export_node_health(self, rate_hz: float, is_connected: bool) -> Dict[str, Any]:
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