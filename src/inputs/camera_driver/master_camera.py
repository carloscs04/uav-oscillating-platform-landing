# File: src/inputs/camera_driver/master_camera.py

from typing import Tuple
from src.inputs.camera_driver.camera_state import CameraState


class CameraManager:
    """Facade API & Lifecycle Manager for the onboard Camera stream.

    Frame capture runs inside CameraNode (attached by MasterNode), which opens
    the camera from config.yaml and writes frames, fps and resolution into
    `state`. Without an attached driver, start/stop only set flags (stub
    behaviour, e.g. when OpenCV is missing).
    """

    def __init__(self, camera_index: int = 0, resolution: Tuple[int, int] = (640, 480), target_fps: float = 30.0):
        self.camera_index = camera_index
        self.resolution = resolution
        self.target_fps = target_fps

        self.state = CameraState()
        self.connected: bool = False
        self.running: bool = False
        self._driver = None          # CameraNode, attached by MasterNode

    def attach_driver(self, driver):
        """Attach the real camera driver (CameraNode)."""
        self._driver = driver

    def start(self) -> bool:
        if self.running:
            return self.connected
        self.running = True
        if self._driver:
            # Opens the camera in the background (self-healing retries); the
            # driver sets state.connected / current_rate_hz / resolution itself.
            self._driver.start()
            self.connected = True
        else:
            self.connected = True
            setattr(self.state, "connected", True)
            setattr(self.state, "current_rate_hz", self.target_fps)
            setattr(self.state, "resolution", f"{self.resolution[0]}x{self.resolution[1]}")
        return True

    def stop(self):
        self.running = False
        self.connected = False
        if self._driver:
            self._driver.stop()
        setattr(self.state, "connected", False)
        setattr(self.state, "current_rate_hz", 0.0)

    def is_healthy(self) -> bool:
        if self._driver:
            return self.running and self._driver.is_running and getattr(self.state, "connected", False)
        return self.running and self.connected

    # --- FACADE WRAPPERS ---
    # With the real driver, resolution and fps come from config.yaml (the
    # calibration is only valid for one resolution), so these only update the
    # stub values.

    def set_resolution(self, width: int, height: int):
        self.resolution = (width, height)
        if not self._driver:
            setattr(self.state, "resolution", f"{width}x{height}")

    def set_fps(self, fps: float):
        self.target_fps = max(1.0, float(fps))
        if not self._driver:
            setattr(self.state, "current_rate_hz", self.target_fps)

    def get_latest_frame(self):
        return getattr(self.state, "latest_b64_frame", None)
