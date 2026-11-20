# file: src/interfaces/MAVLink/state/vehicle_state.py
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
        "LOCAL_POSITION_NED",
        "OPTICAL_FLOW",
        "OPTICAL_FLOW_RAD",
        "DISTANCE_SENSOR",
        "EKF_STATUS_REPORT",
        "VIBRATION",
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
        "VFR_HUD",
        "LOCAL_POSITION_NED",
        "OPTICAL_FLOW",
        "OPTICAL_FLOW_RAD",
        "DISTANCE_SENSOR",
        "EKF_STATUS_REPORT",
        "NAV_CONTROLLER_OUTPUT",
        "SERVO_OUTPUT_RAW",
        "VIBRATION",
        "RC_CHANNELS"
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

    def export_telemetry_data(self) -> Dict[str, Any]:
        """Exports live telemetry packets without altering existing UI keys."""
        att = self.get_packet("ATTITUDE") or {}
        sys_status = self.get_packet("SYS_STATUS") or {}
        hb = self.get_packet("HEARTBEAT") or {}
        vfr = self.get_packet("VFR_HUD") or {}
        pos = self.get_packet("GLOBAL_POSITION_INT") or {}
        local_pos = self.get_packet("LOCAL_POSITION_NED") or {}
        flow = self.get_packet("OPTICAL_FLOW") or {}
        flow_rad = self.get_packet("OPTICAL_FLOW_RAD") or {}
        dist = self.get_packet("DISTANCE_SENSOR") or {}
        ekf = self.get_packet("EKF_STATUS_REPORT") or {}
        nav = self.get_packet("NAV_CONTROLLER_OUTPUT") or {}
        servo = self.get_packet("SERVO_OUTPUT_RAW") or {}
        vib = self.get_packet("VIBRATION") or {}
        rc = self.get_packet("RC_CHANNELS") or {}
        statustext = self.get_packet("STATUSTEXT") or {}

        base_mode = hb.get("base_mode", 0)
        is_armed = bool(base_mode & 128)

        return {
            # Original 4 packets preserved for existing UI
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
                "rollspeed": round(math.degrees(att.get("rollspeed", 0.0)), 2),
                "pitchspeed": round(math.degrees(att.get("pitchspeed", 0.0)), 2),
                "yawspeed": round(math.degrees(att.get("yawspeed", 0.0)), 2),
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
            },
            # Extended navigation & debug packets
            "local_position": {
                "x": round(local_pos.get("x", 0.0), 4),
                "y": round(local_pos.get("y", 0.0), 4),
                "z": round(local_pos.get("z", 0.0), 4),
                "vx": round(local_pos.get("vx", 0.0), 4),
                "vy": round(local_pos.get("vy", 0.0), 4),
                "vz": round(local_pos.get("vz", 0.0), 4),
            },
            "optical_flow": {
                "flow_x": flow.get("flow_x", 0),
                "flow_y": flow.get("flow_y", 0),
                "comp_m_x": round(flow.get("flow_comp_m_x", 0.0), 4),
                "comp_m_y": round(flow.get("flow_comp_m_y", 0.0), 4),
                "quality": flow.get("quality", flow_rad.get("quality", 0)),
                "ground_distance": round(flow.get("ground_distance", 0.0), 3),
                "int_x": round(flow_rad.get("integrated_x", 0.0), 4),
                "int_y": round(flow_rad.get("integrated_y", 0.0), 4),
            },
            "distance_sensor": {
                "current_cm": dist.get("current_distance", 0),
                "min_cm": dist.get("min_distance", 0),
                "max_cm": dist.get("max_distance", 0),
            },
            "ekf_status": {
                "flags": ekf.get("flags", 0),
                "vel_var": round(ekf.get("velocity_variance", 0.0), 4),
                "pos_h_var": round(ekf.get("pos_horiz_variance", 0.0), 4),
                "pos_v_var": round(ekf.get("pos_vert_variance", 0.0), 4),
                "compass_var": round(ekf.get("compass_variance", 0.0), 4),
            },
            "nav_output": {
                "nav_roll": round(nav.get("nav_roll", 0.0), 2),
                "nav_pitch": round(nav.get("nav_pitch", 0.0), 2),
                "nav_bearing": nav.get("nav_bearing", 0),
                "target_bearing": nav.get("target_bearing", 0),
                "wp_dist": nav.get("wp_dist", 0),
                "alt_error": round(nav.get("alt_error", 0.0), 3),
            },
            "servo_output": {
                "m1": servo.get("servo1_raw", 0),
                "m2": servo.get("servo2_raw", 0),
                "m3": servo.get("servo3_raw", 0),
                "m4": servo.get("servo4_raw", 0),
            },
            "vibration": {
                "vib_x": round(vib.get("vibration_x", 0.0), 3),
                "vib_y": round(vib.get("vibration_y", 0.0), 3),
                "vib_z": round(vib.get("vibration_z", 0.0), 3),
                "clip_0": vib.get("clipping_0", 0),
                "clip_1": vib.get("clipping_1", 0),
                "clip_2": vib.get("clipping_2", 0),
            },
            "rc_channels": {
                "roll_ch1": rc.get("chan1_raw", 0),
                "pitch_ch2": rc.get("chan2_raw", 0),
                "throttle_ch3": rc.get("chan3_raw", 0),
                "yaw_ch4": rc.get("chan4_raw", 0),
                "mode_ch5": rc.get("chan5_raw", 0),
            },
            "statustext": {
                "severity": statustext.get("severity", -1),
                "text": statustext.get("text", ""),
            }
        }

    def export_node_health(self, rate_hz: float, is_connected: bool) -> Dict[str, Any]:
        """Unchanged node health export used by Ground Station NodeHealthView."""
        if not is_connected:
            return {
                "status": "RECONNECTING",
                "rate_hz": 0.0,
                "battery_pct": -1,
                "voltage_v": 0.0
            }

        health_status = "HEALTHY" if rate_hz >= 7.0 else ("POOR" if rate_hz > 0.0 else "RECONNECTING")
        sys_status = self.get_packet("SYS_STATUS") or {}

        return {
            "status": health_status,
            "rate_hz": rate_hz,
            "battery_pct": sys_status.get("battery_remaining", -1),
            "voltage_v": round(sys_status.get("voltage_battery", 0) / 1000.0, 2)
        }