# File: src/inputs/distance_sensor/sensor_state.py

import threading
from typing import Any, Dict, Optional


class DistanceSensorState:
    """Thread-safe state buffer for Distance Sensor telemetry and health status."""

    def __init__(self):
        self._lock = threading.Lock()
        self.distance_m: float = 0.0
        self.min_distance_m: float = 0.2
        self.max_distance_m: float = 4.0
        self.sensor_status: str = "OK"

    def update_distance(self, dist_m: float, status: str = "OK"):
        with self._lock:
            self.distance_m = round(dist_m, 3)
            self.sensor_status = status

    def export_telemetry_data(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "distance_sensor": {
                    "distance_m": self.distance_m,
                    "min_distance_m": self.min_distance_m,
                    "max_distance_m": self.max_distance_m,
                    "status": self.sensor_status
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

        health_status = "HEALTHY" if rate_hz >= 5.0 else ("POOR" if rate_hz > 0.0 else "RECONNECTING")
        return {
            "status": health_status,
            "rate_hz": round(rate_hz, 1),
            "battery_pct": -1,
            "voltage_v": 0.0
        }