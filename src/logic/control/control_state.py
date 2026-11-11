# File: src/logic/control/control_state.py

import threading
from typing import Any, Dict, List


class ControlState:
    """Thread-safe state buffer for 3-Axis trajectory tracking and PID controller status."""

    def __init__(self):
        self._lock = threading.Lock()
        self.pos_ref: List[float] = [0.0, 0.0, 1.5]
        self.pos_act: List[float] = [0.0, 0.0, 0.0]
        self.pid_error: List[float] = [0.0, 0.0, 0.0]
        self.control_mode: str = "AUTO_LANDING"

    def update_control_vectors(self, ref: List[float], act: List[float]):
        with self._lock:
            if len(ref) >= 3:
                self.pos_ref = [round(v, 3) for v in ref[:3]]
            if len(act) >= 3:
                self.pos_act = [round(v, 3) for v in act[:3]]
            self.pid_error = [
                round(self.pos_act[i] - self.pos_ref[i], 3) for i in range(3)
            ]

    def export_telemetry_data(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "control_state": {
                    "mode": self.control_mode,
                    "pos_ref": self.pos_ref,
                    "pos_act": self.pos_act,
                    "pid_error": self.pid_error
                },
                "pid_output": {
                    "pos_ref": self.pos_ref,
                    "pos_act": self.pos_act
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