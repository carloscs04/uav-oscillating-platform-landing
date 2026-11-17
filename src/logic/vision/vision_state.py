# File: src/logic/vision/vision_state.py

import math
import threading
import time
from typing import Any, Dict, Optional


class VisionState:
    """Thread-safe buffer for ArUco marker detection, 3D pose estimation, and optical flow."""

    def __init__(self):
        self._lock = threading.Lock()
        self.aruco_detected: bool = False
        self.aruco_id: int = -1
        self.aruco_pos: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.aruco_att: Dict[str, float] = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.distance_m: float = 0.0
        self.tracking_state: str = "SEARCHING"
        self.last_detection_ts: float = 0.0

        # Optical flow state buffer (read by MAVLinkSensorFeed at 30Hz)
        self.flow_rad_x: float = 0.0
        self.flow_rad_y: float = 0.0
        self.flow_dt_s: float = 0.033
        self.flow_quality: int = 0
        self.flow_px_dx: float = 0.0
        self.flow_px_dy: float = 0.0
        self.flow_seq: int = 0
        self.last_flow_ts: float = 0.0

    def update_aruco_pose(
        self,
        detected: bool,
        target_id: int = -1,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        tracking_state: Optional[str] = None,
    ):
        """Update ArUco detection state and 3D pose from the vision algorithm."""
        with self._lock:
            self.aruco_detected = detected
            self.aruco_id = target_id if detected else -1
            self.aruco_pos = {
                "x": round(x, 3),
                "y": round(y, 3),
                "z": round(z, 3),
            }
            self.aruco_att = {
                "roll": round(roll, 2),
                "pitch": round(pitch, 2),
                "yaw": round(yaw, 2),
            }
            self.distance_m = round(math.sqrt(x**2 + y**2 + z**2), 3) if detected else 0.0

            if tracking_state is not None:
                self.tracking_state = tracking_state
            else:
                self.tracking_state = "LOCKED" if detected else "SEARCHING"

            if detected:
                self.last_detection_ts = time.time()

    def update_optical_flow(
        self,
        rad_x: float,
        rad_y: float,
        dt_s: float,
        quality: int,
        px_dx: float = 0.0,
        px_dy: float = 0.0,
    ):
        """Update frame-to-frame angular optical flow sample for EKF3 velocity fusion."""
        with self._lock:
            self.flow_rad_x = float(rad_x)
            self.flow_rad_y = float(rad_y)
            self.flow_dt_s = float(dt_s)
            self.flow_quality = int(quality)
            self.flow_px_dx = round(float(px_dx), 2)
            self.flow_px_dy = round(float(px_dy), 2)
            self.flow_seq += 1
            self.last_flow_ts = time.time()

    def get_optical_flow(self) -> Dict[str, Any]:
        """Return latest optical flow sample snapshot."""
        with self._lock:
            return {
                "rad_x": self.flow_rad_x,
                "rad_y": self.flow_rad_y,
                "dt_s": self.flow_dt_s,
                "quality": self.flow_quality,
                "px_dx": self.flow_px_dx,
                "px_dy": self.flow_px_dy,
                "seq": self.flow_seq,
                "ts": self.last_flow_ts,
            }

    def export_telemetry_data(self) -> Dict[str, Any]:
        """Export ArUco pose telemetry, optical flow metrics, and vision pipeline status packets."""
        with self._lock:
            return {
                "aruco_pose": {
                    "detected": self.aruco_detected,
                    "id": self.aruco_id,
                    "x": self.aruco_pos["x"],
                    "y": self.aruco_pos["y"],
                    "z": self.aruco_pos["z"],
                    "roll": self.aruco_att["roll"],
                    "pitch": self.aruco_att["pitch"],
                    "yaw": self.aruco_att["yaw"],
                    "distance_m": self.distance_m,
                },
                "optical_flow": {
                    "rad_x": round(self.flow_rad_x, 5),
                    "rad_y": round(self.flow_rad_y, 5),
                    "quality": self.flow_quality,
                    "px_dx": self.flow_px_dx,
                    "px_dy": self.flow_px_dy,
                },
                "status": {
                    "target_locked": self.aruco_detected,
                    "tracking_state": self.tracking_state,
                    "last_seen_ts": round(self.last_detection_ts, 2),
                },
            }

    def export_node_health(self, rate_hz: float, is_connected: bool) -> Dict[str, Any]:
        """Export health status for the Vision-ArUco node."""
        if not is_connected:
            return {
                "status": "RECONNECTING",
                "rate_hz": 0.0,
                "battery_pct": -1,
                "voltage_v": 0.0,
            }

        health_status = "HEALTHY" if rate_hz >= 10.0 else ("POOR" if rate_hz > 0.0 else "RECONNECTING")
        return {
            "status": health_status,
            "rate_hz": round(rate_hz, 1),
            "battery_pct": -1,
            "voltage_v": 0.0,
        }