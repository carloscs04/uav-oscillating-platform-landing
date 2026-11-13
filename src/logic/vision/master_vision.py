# File: src/logic/vision/master_vision.py

import threading
import time
from typing import Any, Dict, Optional
from src.logic.vision.vision_state import VisionState


class VisionManager:
    """Facade API & Lifecycle Manager for ArUco detection and relative pose estimation."""

    def __init__(self, target_marker_id: int = 42, target_hz: float = 30.0):
        self.target_marker_id = target_marker_id
        self.target_hz = target_hz

        self.state = VisionState()
        self.connected: bool = False
        self.running: bool = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        if self.running:
            return self.connected
        self.running = True
        self.connected = True
        setattr(self.state, "connected", True)
        setattr(self.state, "current_rate_hz", self.target_hz)

        self._thread = threading.Thread(target=self._vision_loop, daemon=True)
        self._thread.start()
        return True

    def _vision_loop(self):
        """Background ArUco detection loop hook."""
        interval = 1.0 / max(1.0, self.target_hz)
        while self.running:
            time.sleep(interval)

    def stop(self):
        self.running = False
        self.connected = False
        setattr(self.state, "connected", False)
        setattr(self.state, "current_rate_hz", 0.0)

    def is_healthy(self) -> bool:
        return self.running and self.connected

    # --- FACADE WRAPPERS ---

    def set_target_marker(self, marker_id: int):
        self.target_marker_id = int(marker_id)

    def is_target_locked(self) -> bool:
        return bool(getattr(self.state, "aruco_detected", False))

    def get_target_pose(self) -> Dict[str, Any]:
        return {
            "detected": self.is_target_locked(),
            "id": getattr(self.state, "aruco_id", -1),
            "x": getattr(self.state, "x", 0.0),
            "y": getattr(self.state, "y", 0.0),
            "z": getattr(self.state, "z", 0.0),
            "yaw": getattr(self.state, "yaw", 0.0),
        }