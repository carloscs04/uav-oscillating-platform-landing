# File: src/inputs/distance_sensor/sensor_state.py

import threading
import time
from typing import Any, Callable, Dict, Optional

from src.logic.control.lidar_gate import LidarGate


class DistanceState:
    """Thread-safe state buffer for Distance Sensor telemetry and health status."""

    def __init__(self):
        self._lock = threading.Lock()
        self.distance_m: float = 0.0
        self.min_distance_m: float = 0.2
        self.max_distance_m: float = 4.0
        self.sensor_status: str = "OK"
        # Spike gate (src/logic/control/lidar_gate.py): distance_m is the GATED height that
        # the Pixhawk, the camera and the landing law use; raw_distance_m is the sensor.
        self.raw_distance_m: float = 0.0
        self.gate = LidarGate()
        self._vz_up_fn: Optional[Callable[[], Optional[float]]] = None
        self._cam_h_fn: Optional[Callable[[], Optional[float]]] = None
        self._active_fn: Optional[Callable[[], bool]] = None
        self.test_until: float = 0.0   # `lidargate test`: gate active on the bench (disarmed only)

    def set_gate_inputs(self, vz_up_fn=None, cam_h_fn=None, active_fn=None, on_event=None):
        """vz_up_fn() -> Pixhawk vertical speed m/s (+ up) or None; cam_h_fn() -> camera-only
        ArUco height m or None; active_fn() -> True when the gate may act (armed in LOITER or
        GUIDED; None = always); on_event(text) -> one line per decision change."""
        self._vz_up_fn, self._cam_h_fn, self._active_fn = vz_up_fn, cam_h_fn, active_fn
        self.gate.on_event = on_event

    @staticmethod
    def _safe(fn):
        try:
            return fn() if fn is not None else None
        except Exception:
            return None

    def update_distance(self, dist_m: float, status: str = "OK"):
        vz = cam = None
        active = True
        if status == "OK":   # providers read other states: call them outside our lock
            vz, cam = self._safe(self._vz_up_fn), self._safe(self._cam_h_fn)
            active = self._active_fn is None or bool(self._safe(self._active_fn))
        with self._lock:
            self.raw_distance_m = round(dist_m, 3)
            if status != "OK":
                self.distance_m, self.sensor_status = round(dist_m, 3), status
                return
            before = self.gate.status
            if active:
                out = self.gate.step(float(dist_m), time.monotonic(), vz, cam)
            else:
                out = self.gate.idle(float(dist_m), time.monotonic())
            if self.gate.status != before:
                print(f"[LIDARGATE] {before} -> {self.gate.status} (raw {dist_m * 100:.1f} cm, "
                      f"used {'none' if out is None else f'{out * 100:.1f} cm'})", flush=True)
            if out is None:     # UNRELIABLE: consumers see "no lidar" (keep the last value)
                self.sensor_status = "UNRELIABLE"
            else:
                self.distance_m, self.sensor_status = round(out, 3), "OK"

    def gate_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            d = self.gate.snapshot()
            d["raw_m"] = self.raw_distance_m
            d["sensor_status"] = self.sensor_status
            d["event_log"] = [list(e) for e in self.gate.event_log]
            return d

    def export_telemetry_data(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "distance_sensor": {
                    "distance_m": self.distance_m,
                    "min_distance_m": self.min_distance_m,
                    "max_distance_m": self.max_distance_m,
                    "status": self.sensor_status,
                    "raw_m": self.raw_distance_m,
                    "gate": self.gate.status,
                    "gate_source": self.gate.source,
                    "gate_rejected": self.gate.rejected_total,
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