# File: src/logic/master/master_state.py

import threading
import time
from enum import Enum
from typing import Any, Dict, List


class SystemState(str, Enum):
    """Global UAV operational state machine."""
    BOOTING = "BOOTING"
    IDLE = "IDLE"
    READY = "READY"
    MISSION_ACTIVE = "MISSION_ACTIVE"
    LANDING = "LANDING"
    EMERGENCY_ABORT = "EMERGENCY_ABORT"


class MasterState:
    """Thread-safe state container for MasterNode orchestration and CLI console streaming."""

    def __init__(self):
        self._lock = threading.Lock()
        self.start_time: float = time.time()
        self.system_state: SystemState = SystemState.IDLE
        self.armed: bool = False
        self.connected: bool = True
        self.current_rate_hz: float = 10.0
        self.boot_id: int = int(time.time() * 1000)

        # Registry of managed subsystem states
        self.subsystems: Dict[str, Dict[str, Any]] = {
            "mavlink": {"running": False, "health": "OFFLINE", "mode": "HARDWARE"},
            "camera": {"running": False, "health": "OFFLINE"},
            "vision": {"running": False, "health": "OFFLINE"},
            "distance": {"running": False, "health": "OFFLINE"},
            "control": {"running": False, "health": "OFFLINE"},
        }

        # Sequenced ring-buffer for remote SSH-like terminal streaming
        self._log_seq: int = 0
        self._cli_logs: List[Dict[str, Any]] = []
        self._max_logs: int = 40

    def set_system_state(self, new_state: SystemState):
        with self._lock:
            self.system_state = new_state

    def get_system_state(self) -> SystemState:
        with self._lock:
            return self.system_state

    def set_armed(self, armed: bool):
        with self._lock:
            self.armed = armed

    def is_armed(self) -> bool:
        with self._lock:
            return self.armed

    def update_subsystem(self, name: str, running: bool, health: str = "HEALTHY"):
        with self._lock:
            if name in self.subsystems:
                self.subsystems[name]["running"] = running
                self.subsystems[name]["health"] = health if running else "OFFLINE"

    def get_subsystems_snapshot(self) -> Dict[str, Dict[str, Any]]:
        with self._lock:
            return {k: dict(v) for k, v in self.subsystems.items()}

    def append_cli_log(self, message: str, level: str = "INFO"):
        """Prints locally to SSH stdout and appends sequenced line to UDP telemetry buffer."""
        timestamp = time.strftime("%H:%M:%S")
        formatted = f"[{timestamp}] [{level}] {message}"
        print(formatted, flush=True)

        with self._lock:
            self._log_seq += 1
            self._cli_logs.append({
                "seq": self._log_seq,
                "text": formatted
            })
            if len(self._cli_logs) > self._max_logs:
                self._cli_logs.pop(0)

    def export_telemetry_data(self) -> Dict[str, Any]:
        """Exports Master status and sequenced CLI stream packets over UDP port 5001."""
        with self._lock:
            return {
                "master_status": {
                    "system_state": self.system_state.value,
                    "armed": self.armed,
                    "uptime_sec": int(time.time() - self.start_time),
                    "subsystems": {k: v["running"] for k, v in self.subsystems.items()}
                },
                "cli_stream": {
                    "boot_id": self.boot_id,
                    "latest_seq": self._log_seq,
                    "lines": list(self._cli_logs[-20:])
                },
                "status": {
                    "state": self.system_state.value,
                    "active_nodes": sum(1 for v in self.subsystems.values() if v["running"]),
                    "total_nodes": len(self.subsystems)
                }
            }

    def export_node_health(self, rate_hz: float, is_connected: bool) -> Dict[str, Any]:
        """Exports Master Node health metadata."""
        return {
            "status": "HEALTHY" if is_connected else "OFFLINE",
            "rate_hz": round(rate_hz, 1),
            "battery_pct": -1,
            "voltage_v": 0.0
        }