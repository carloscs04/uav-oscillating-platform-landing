# File: src/logic/vision/master_vision.py

from typing import Any, Dict
from src.logic.vision.vision_state import VisionState


class VisionManager:
    """Facade API & Lifecycle Manager for ArUco detection and relative pose estimation.

    The ArUco pipeline itself runs inside CameraNode (same frames as the video
    stream); this manager only switches it on/off. Without an attached driver,
    start/stop only set flags (stub behaviour, e.g. when OpenCV is missing).
    """

    def __init__(self, target_marker_id: int = 32, target_hz: float = 30.0):
        self.target_marker_id = target_marker_id
        self.target_hz = target_hz

        self.state = VisionState()
        self.connected: bool = False
        self.running: bool = False
        self._driver = None          # CameraNode, attached by MasterNode

    def attach_driver(self, driver):
        """Attach the real ArUco pipeline (CameraNode)."""
        self._driver = driver

    def start(self) -> bool:
        if self.running:
            return self.connected
        self.running = True
        if self._driver:
            # The driver sets state.connected / current_rate_hz from real frames.
            self._driver.enable_vision(True)
            self.connected = self._driver.is_running
            if not self._driver.is_running:
                print("[VISION] Vision needs the camera: run 'start camera'.")
        else:
            self.connected = True
            setattr(self.state, "connected", True)
            setattr(self.state, "current_rate_hz", self.target_hz)
        return True

    def stop(self):
        self.running = False
        self.connected = False
        if self._driver:
            self._driver.enable_vision(False)
        setattr(self.state, "connected", False)
        setattr(self.state, "current_rate_hz", 0.0)

    def is_healthy(self) -> bool:
        if self._driver:
            return self.running and self._driver.is_running
        return self.running and self.connected

    # --- FACADE WRAPPERS ---

    def set_target_marker(self, marker_id: int):
        self.target_marker_id = int(marker_id)

    def is_target_locked(self) -> bool:
        return bool(getattr(self.state, "aruco_detected", False))

    def get_target_pose(self) -> Dict[str, Any]:
        pos = getattr(self.state, "aruco_pos", {}) or {}
        att = getattr(self.state, "aruco_att", {}) or {}
        return {
            "detected": self.is_target_locked(),
            "id": getattr(self.state, "aruco_id", -1),
            "x": pos.get("x", 0.0),
            "y": pos.get("y", 0.0),
            "z": pos.get("z", 0.0),
            "yaw": att.get("yaw", 0.0),
            "distance_m": getattr(self.state, "distance_m", 0.0),
        }
