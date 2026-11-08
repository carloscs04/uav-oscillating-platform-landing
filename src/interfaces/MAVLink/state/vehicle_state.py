import threading
from typing import Any, Dict, List, Optional, Set
from pymavlink import mavutil
import math

class VehicleState:
    """Thread-safe telemetry buffer storing live packet updates and watch lists."""

    KNOWN_PACKETS = [
        "HEARTBEAT",
        "STATUSTEXT",
        "COMMAND_ACK",
        "ATTITUDE",
        "VFR_HUD",
        "SYS_STATUS",
        "GLOBAL_POSITION_INT",
        "POWER_STATUS",
        "HIGHRES_IMU",
        "NAV_CONTROLLER_OUTPUT",
        "SERVO_OUTPUT_RAW",
        "RC_CHANNELS"
    ]

    DEFAULT_WATCHED_PACKETS = [
        "HEARTBEAT",
        "COMMAND_ACK",
        "STATUSTEXT",
        "SYS_STATUS",
        "ATTITUDE",
        "VFR_HUD"
    ]

    def __init__(self):
        self._lock = threading.Lock()
        self._watched_packets: Set[str] = set(self.DEFAULT_WATCHED_PACKETS)
        self._latest_telemetry_cache: Dict[str, Dict[str, Any]] = {}

    def update_packet(self, pkt_type: str, data: Dict[str, Any]):
        with self._lock:
            self._latest_telemetry_cache[pkt_type] = data

    def get_packet(self, pkt_type: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            return self._latest_telemetry_cache.get(pkt_type)

    def is_armed(self) -> bool:
        """Returns True if the latest HEARTBEAT indicates the drone is armed."""
        hb = self.get_packet("HEARTBEAT")
        if hb:
            base_mode = hb.get("base_mode", 0)
            return bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return False

    def clear_transient_packets(self):
        """Flushes stale command receipts before executing new commands."""
        with self._lock:
            self._latest_telemetry_cache.pop("COMMAND_ACK", None)
            self._latest_telemetry_cache.pop("STATUSTEXT", None)

    def add_watched_packet(self, pkt_name: str) -> bool:
        with self._lock:
            self._watched_packets.add(pkt_name.upper())
        return True

    def remove_watched_packet(self, pkt_name: str) -> bool:
        pkt_upper = pkt_name.upper()
        with self._lock:
            if pkt_upper in self._watched_packets:
                self._watched_packets.remove(pkt_upper)
                return True
        return False

    def get_watched_packets(self) -> List[str]:
        with self._lock:
            return sorted(list(self._watched_packets))

    def get_available_unwatched_packets(self) -> List[str]:
        with self._lock:
            return sorted([pkt for pkt in self.KNOWN_PACKETS if pkt not in self._watched_packets])

    def get_velocity(self) -> Dict[str, float]:
        data = self.get_packet("GLOBAL_POSITION_INT")
        if data:
            return {
                "vx": round(data.get("vx", 0) / 100.0, 3),
                "vy": round(data.get("vy", 0) / 100.0, 3),
                "vz": round(data.get("vz", 0) / 100.0, 3),
            }
        return {"vx": 0.0, "vy": 0.0, "vz": 0.0}

    def export_telemetry_package(self) -> Dict[str, Any]:
        """Exports all live telemetry formatted into clean, pre-structured JSON-ready packages."""
        att = self.get_packet("ATTITUDE") or {}
        sys_status = self.get_packet("SYS_STATUS") or {}
        hb = self.get_packet("HEARTBEAT") or {}
        vfr = self.get_packet("VFR_HUD") or {}
        pos = self.get_packet("GLOBAL_POSITION_INT") or {}

        base_mode = hb.get("base_mode", 0)
        is_armed = bool(base_mode & 128)

        return {
            "velocity": {
                "vx": round(pos.get("vx", 0) / 100.0, 3),
                "vy": round(pos.get("vy", 0) / 100.0, 3),
                "vz": round(pos.get("vz", 0) / 100.0, 3),
                "groundspeed": round(vfr.get("groundspeed", 0.0), 2),
                "heading": vfr.get("heading", 0),
            },
            "attitude": {
                "roll": round(math.degrees(att.get("roll", 0.0)), 2),
                "pitch": round(math.degrees(att.get("pitch", 0.0)), 2),
                "yaw": round(math.degrees(att.get("yaw", 0.0)), 2),
            },
            "sys_status": {
                "voltage_v": round(sys_status.get("voltage_battery", 0) / 1000.0, 2),
                "current_a": round(sys_status.get("current_battery", 0) / 100.0, 2),
                "battery_pct": sys_status.get("battery_remaining", -1),
                "cpu_load_pct": round(sys_status.get("load", 0) / 10.0, 1),
            },
            "heartbeat": {
                "armed": is_armed,
                "base_mode": base_mode,
                "custom_mode": hb.get("custom_mode", 0),
            }
        }