# file: src/interfaces/MAVLink/state/vehicle_state.py
import collections
import threading
import time
from typing import Any, Dict, List, Optional, Set
from pymavlink import mavutil
import math


class VehicleState:
    """Thread-safe telemetry buffer storing live packet updates, watch lists, and Pixhawk parameters."""

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
        "SCALED_PRESSURE",
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
        "SCALED_PRESSURE",
        "RC_CHANNELS"
    ]

    def __init__(self):
        self._lock = threading.Lock()
        self._watched_packets: Set[str] = set(self.DEFAULT_WATCHED_PACKETS)
        self._latest_telemetry_cache: Dict[str, Dict[str, Any]] = {}
        self._rx_time: Dict[str, float] = {}
        self._pixhawk_params: Dict[str, float] = {}
        # Pi-side control gains (landing law + GUIDED sender), read through a callable so
        # the ground station events log records the values actually flying.
        self._control_params_fn = None
        # Every STATUSTEXT / COMMAND_ACK in arrival order (the cache above keeps only the
        # latest one, and ArduPilot sends PreArm reasons as several texts in a row).
        self._event_seq: int = 0
        self._events: collections.deque = collections.deque(maxlen=200)
        # Pi-side sensor feed (OPTICAL_FLOW we send), exported for the ground station CSV
        self._sensor_feed: Any = None
        # Barometer altitude relative to the ground pressure, which is averaged while
        # DISARMED and frozen while ARMED (= height above the take-off point). Logged
        # to compare against the lidar before trusting the baro as the height source.
        self._baro_ref_hpa: Optional[float] = None
        self._baro_rel_alt_m: Optional[float] = None
        # EKF source set in use: 1 = lidar height, 2 = baro height, 0 = unknown.
        # ArduPilot does not report it, so it is tracked from our own ACKed commands and
        # reset to 1 when the Pixhawk reboots (every boot starts with source set 1).
        self._height_source_set: int = 0
        self._last_boot_ms: Optional[int] = None

    EVENT_PACKETS = ("STATUSTEXT", "COMMAND_ACK")

    def update_packet(self, pkt_type: str, data: Dict[str, Any]):
        with self._lock:
            self._latest_telemetry_cache[pkt_type] = data
            self._rx_time[pkt_type] = time.time()
            if pkt_type == "SCALED_PRESSURE":
                self._update_baro_locked(data)
            elif pkt_type == "ATTITUDE":
                boot_ms = int(data.get("time_boot_ms", 0) or 0)
                if self._last_boot_ms is not None and boot_ms + 5000 < self._last_boot_ms:
                    self._height_source_set = 1   # Pixhawk rebooted -> source set 1 (lidar)
                self._last_boot_ms = boot_ms
            if pkt_type in self.EVENT_PACKETS:
                self._event_seq += 1
                self._events.append((self._event_seq, time.time(), pkt_type, data))

    BARO_REF_ALPHA = 0.05   # ~2 s smoothing at 10 Hz while disarmed

    def _update_baro_locked(self, data: Dict[str, Any]):
        p = float(data.get("press_abs", 0.0) or 0.0)
        if p <= 0.0:
            return
        hb = self._latest_telemetry_cache.get("HEARTBEAT") or {}
        armed = bool(hb.get("base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if self._baro_ref_hpa is None:
            self._baro_ref_hpa = p
        elif not armed:
            self._baro_ref_hpa += self.BARO_REF_ALPHA * (p - self._baro_ref_hpa)
        self._baro_rel_alt_m = 44330.77 * (1.0 - (p / self._baro_ref_hpa) ** 0.190263)

    HEIGHT_SOURCE_NAMES = {0: "UNKNOWN", 1: "LIDAR", 2: "BARO", 3: "SET3"}

    def set_height_source_set(self, source_set: int):
        with self._lock:
            self._height_source_set = int(source_set)

    def get_height_source_set(self) -> int:
        with self._lock:
            return self._height_source_set

    def reset_baro_reference(self):
        """Next SCALED_PRESSURE becomes the new zero (used after 'calibrate baro')."""
        with self._lock:
            self._baro_ref_hpa = None

    def _export_baro(self) -> Dict[str, Any]:
        with self._lock:
            d = self._latest_telemetry_cache.get("SCALED_PRESSURE") or {}
            ref, alt = self._baro_ref_hpa, self._baro_rel_alt_m
        return {
            "press_hpa": round(float(d.get("press_abs", 0.0) or 0.0), 3),
            "temp_c": round(float(d.get("temperature", 0) or 0) / 100.0, 2),
            "rel_alt_m": round(alt, 3) if alt is not None else 0.0,
            "ref_hpa": round(ref, 3) if ref is not None else 0.0,
            "age_ms": self._age_ms("SCALED_PRESSURE"),
        }

    def get_events_since(self, seq: int) -> List[tuple]:
        """(seq, rx_time, pkt_type, data) for every STATUSTEXT/COMMAND_ACK newer than seq."""
        with self._lock:
            return [e for e in self._events if e[0] > seq]

    def last_event_seq(self) -> int:
        with self._lock:
            return self._event_seq

    def attach_sensor_feed(self, sensor_feed: Any):
        self._sensor_feed = sensor_feed

    def _export_pi_flow(self) -> Dict[str, Any]:
        """What the Pi sends to the Pixhawk: raw camera flow (rad/s), quality, lidar height."""
        feed = self._sensor_feed
        f = dict(getattr(feed, "last_flow", None) or {}) if feed else {}
        h = f.get("height_m")
        t = f.get("t")
        return {
            "rate_x": round(float(f.get("raw_rate_x", 0.0)), 5),
            "rate_y": round(float(f.get("raw_rate_y", 0.0)), 5),
            "quality": int(f.get("raw_quality", 0)),
            "sent_quality": int(f.get("quality", 0)),
            "gated": int(bool(f.get("gated", False))),
            "lidar_m": round(float(h), 3) if h is not None else -1.0,
            **self._export_lidar_gate(feed),
            "samples": int(f.get("samples", 0) or 0),
            "age_ms": int((time.time() - t) * 1000) if t else -1,
            "tx_hz": round(float(getattr(feed, "actual_rate_hz", 0.0) or 0.0), 1) if feed else 0.0,
        }

    @staticmethod
    def _export_lidar_gate(feed) -> Dict[str, Any]:
        """Raw lidar and the spike gate status (DistanceState), for the flight CSV."""
        st = getattr(feed, "distance_state", None) if feed else None
        snap = st.gate_snapshot() if st is not None and hasattr(st, "gate_snapshot") else {}
        return {"lidar_raw_m": snap.get("raw_m", -1.0), "lidar_gate": snap.get("status", ""),
                "lidar_src": snap.get("source", ""),
                "lidar_cam_m": snap.get("cam_m") if snap.get("cam_m") is not None else -1.0,
                "lidar_rejected": snap.get("rejected_total", 0)}

    def _export_lidargate_log(self) -> List[list]:
        """Last lidar spike gate decisions [seq, text] (flight events log, like STATUSTEXT)."""
        feed = self._sensor_feed
        st = getattr(feed, "distance_state", None) if feed else None
        if st is None or not hasattr(st, "gate_snapshot"):
            return []
        return st.gate_snapshot().get("event_log", [])[-10:]

    def _export_statustext_log(self, n: int = 10) -> List[list]:
        with self._lock:
            evs = [e for e in self._events if e[2] == "STATUSTEXT"][-n:]
        return [[seq, d.get("severity", -1), str(d.get("text", ""))] for seq, _t, _k, d in evs]

    def get_packet_age(self, pkt_type: str) -> Optional[float]:
        """Seconds since pkt_type was last received from the Pixhawk (None = never)."""
        with self._lock:
            t = self._rx_time.get(pkt_type)
        return None if t is None else time.time() - t

    def get_packet_with_time(self, pkt_type: str):
        """(data, receive_time) for pkt_type, or (None, None)."""
        with self._lock:
            return self._latest_telemetry_cache.get(pkt_type), self._rx_time.get(pkt_type)

    def get_packet(self, pkt_type: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            return self._latest_telemetry_cache.get(pkt_type)

    def update_param(self, param_id: str, value: float):
        """Stores a fetched Pixhawk configuration parameter."""
        with self._lock:
            self._pixhawk_params[param_id] = value

    def get_params(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._pixhawk_params)

    def set_control_params_provider(self, fn):
        self._control_params_fn = fn

    def get_control_params(self) -> Dict[str, Any]:
        fn = self._control_params_fn
        if fn is None:
            return {}
        try:
            return dict(fn())
        except Exception:
            return {}

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
        """Exports live telemetry packets and cached Pixhawk parameters without altering existing UI keys."""
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
                "terrain_var": round(ekf.get("terrain_alt_variance", 0.0), 4),
                "age_ms": self._age_ms("EKF_STATUS_REPORT"),
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
                "age_ms": self._age_ms("VIBRATION"),
            },
            "pi_flow": self._export_pi_flow(),
            "baro": self._export_baro(),
            "height_src": {"set": self.get_height_source_set(),
                           "name": self.HEIGHT_SOURCE_NAMES.get(self.get_height_source_set(), "?")},
            "statustext_log": self._export_statustext_log(),
            "lidargate_log": self._export_lidargate_log(),
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
            },
            "pixhawk_params": self.get_params(),
            "control_params": self.get_control_params(),
        }

    def _age_ms(self, pkt_type: str) -> int:
        """ms since pkt_type arrived (-1 = never): shows in the CSV whether a column is live."""
        age = self.get_packet_age(pkt_type)
        return -1 if age is None else int(age * 1000)

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