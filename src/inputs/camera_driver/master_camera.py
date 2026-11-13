# File: src/inputs/camera_driver/master_camera.py

import threading
import time
from typing import Optional, Tuple
from src.inputs.camera_driver.camera_state import CameraState


class CameraManager:
    """Facade API & Lifecycle Manager for the onboard Camera stream."""

    def __init__(self, camera_index: int = 0, resolution: Tuple[int, int] = (640, 480), target_fps: float = 30.0):
        self.camera_index = camera_index
        self.resolution = resolution
        self.target_fps = target_fps

        self.state = CameraState()
        self.connected: bool = False
        self.running: bool = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        if self.running:
            return self.connected
        self.running = True
        self.connected = True
        setattr(self.state, "connected", True)
        setattr(self.state, "current_rate_hz", self.target_fps)
        setattr(self.state, "resolution", f"{self.resolution[0]}x{self.resolution[1]}")

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        return True

    def _capture_loop(self):
        """Background frame acquisition hook (connect OpenCV/Picamera2 driver here)."""
        interval = 1.0 / max(1.0, self.target_fps)
        while self.running:
            setattr(self.state, "fps_actual", self.target_fps)
            time.sleep(interval)

    def stop(self):
        self.running = False
        self.connected = False
        setattr(self.state, "connected", False)
        setattr(self.state, "current_rate_hz", 0.0)

    def is_healthy(self) -> bool:
        return self.running and self.connected

    # --- FACADE WRAPPERS ---

    def set_resolution(self, width: int, height: int):
        self.resolution = (width, height)
        setattr(self.state, "resolution", f"{width}x{height}")

    def set_fps(self, fps: float):
        self.target_fps = max(1.0, float(fps))
        setattr(self.state, "current_rate_hz", self.target_fps)

    def get_latest_frame(self):
        return getattr(self.state, "latest_frame", None)