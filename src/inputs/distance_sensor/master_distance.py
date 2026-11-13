# File: src/inputs/distance_sensor/master_distance.py

import threading
import time
from typing import Optional
from src.inputs.distance_sensor.sensor_state import DistanceSensorState


class DistanceManager:
    """Facade API & Lifecycle Manager for downward rangefinder / LiDAR sensor."""

    def __init__(self, target_hz: float = 15.0):
        self.target_hz = target_hz

        self.state = DistanceSensorState()
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

        self._thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self._thread.start()
        return True

    def _sensor_loop(self):
        """Background rangefinder polling hook (connect I2C/UART TFmini/VL53L1X here)."""
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

    def get_distance_m(self) -> float:
        return float(getattr(self.state, "distance_m", 0.0))

    def inject_reading(self, distance_m: float):
        if hasattr(self.state, "update_distance"):
            self.state.update_distance(distance_m)
        elif hasattr(self.state, "update"):
            self.state.update(distance_m)