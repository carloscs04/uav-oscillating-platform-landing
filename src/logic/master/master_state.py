# File: src/logic/master/master_state.py

import threading
import time
from typing import Any, Dict, List


class MasterState:
    """Thread-safe state buffer for Master System orchestrator logs and CPU/RAM status."""

    def __init__(self):
        self._lock = threading.Lock()
        self.terminal_logs: List[str] = []
        self.cpu_usage_pct: float = 12.5
        self.ram_usage_pct: float = 45.0
        self.system_status: str = "RUNNING"

    def append_terminal_log(self, log_msg: str):
        with self._lock:
            timestamp = time.strftime("%H:%M:%S")
            self.terminal_logs.append(f"[{timestamp}] {log_msg}")
            if len(self.terminal_logs) > 50:
                self.terminal_logs.pop(0)

    def update_system_metrics(self, cpu: float, ram: float, status: str = "RUNNING"):
        with self._lock:
            self.cpu_usage_pct = round(cpu, 1)
            self.ram_usage_pct = round(ram, 1)
            self.system_status = status

    def export_telemetry_data(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "terminal_output": {
                    "logs": list(self.terminal_logs)
                },
                "system_status": {
                    "state": self.system_status,
                    "cpu_pct": self.cpu_usage_pct,
                    "ram_pct": self.ram_usage_pct
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