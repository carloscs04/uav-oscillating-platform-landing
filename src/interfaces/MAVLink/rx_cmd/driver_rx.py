# src/interfaces/MAVLink/rx_cmd/driver_rx.py

import math
import time
from typing import Any, Dict, List, Optional, Set
from pymavlink import mavutil


class MAVLinkRXDriver:
    """MAVLink Receiver Driver: Exclusively handles high-frequency telemetry reading, 

    parsing, dynamic packet watch lists, and clean visualization formatting.
    """

    # Catalog of known telemetry packet types for dynamic assignment
    KNOWN_PACKETS = [
        "ATTITUDE",
        "VFR_HUD",
        "SYS_STATUS",
        "GLOBAL_POSITION_INT",
        "POWER_STATUS",
        "HIGHRES_IMU",
        "NAV_CONTROLLER_OUTPUT",
        "SERVO_OUTPUT_RAW",
        "RC_CHANNELS",
        "HEARTBEAT"
    ]

    # Default organized packets to display on startup
    DEFAULT_WATCHED_PACKETS = [
        "ATTITUDE",
        "VFR_HUD",
        "SYS_STATUS",
        "GLOBAL_POSITION_INT"
    ]

    def __init__(self, master: Optional[Any] = None):
        self.master = master
        self.watched_packets: Set[str] = set(self.DEFAULT_WATCHED_PACKETS)
        self.latest_telemetry_cache: Dict[str, Dict[str, Any]] = {}

    def set_master(self, master: Any):
        """Assigns the shared MAVLink hardware connection instance."""
        self.master = master

    def add_watched_packet(self, pkt_name: str) -> bool:
        """Adds a packet type to the active visualization watch list."""
        pkt_upper = pkt_name.upper()
        self.watched_packets.add(pkt_upper)
        return True

    def remove_watched_packet(self, pkt_name: str) -> bool:
        """Removes a packet type from the active visualization watch list."""
        pkt_upper = pkt_name.upper()
        if pkt_upper in self.watched_packets:
            self.watched_packets.remove(pkt_upper)
            return True
        return False

    def get_watched_packets(self) -> List[str]:
        """Returns the current list of active watched packet types."""
        return sorted(list(self.watched_packets))

    def get_available_unwatched_packets(self) -> List[str]:
        """Returns known MAVLink packet types that are not currently watched."""
        return sorted([pkt for pkt in self.KNOWN_PACKETS if pkt not in self.watched_packets])

    def wait_for_heartbeat(self, timeout: float = 5.0) -> Optional[Dict[str, Any]]:
        """Waits for an incoming heartbeat packet from the flight controller."""
        if not self.master:
            return None
        hb = self.master.wait_heartbeat(timeout=timeout)
        return {"system_id": self.master.target_system} if hb else None

    def read_telemetry(self) -> Optional[Dict[str, Any]]:
        """Reads non-blocking incoming telemetry messages at full hardware speed."""
        if not self.master:
            return None
        msg = self.master.recv_match(blocking=False)
        if msg:
            data = msg.to_dict()
            pkt_type = data.get("mavpackettype")
            if pkt_type:
                self.latest_telemetry_cache[pkt_type] = data
            return data
        return None

    def format_packet_summary(self, pkt_type: str, data: Dict[str, Any]) -> str:
        """Transforms raw dictionary data into clean, human-readable single-line summaries."""
        if pkt_type == "ATTITUDE":
            roll_deg = math.degrees(data.get("roll", 0.0))
            pitch_deg = math.degrees(data.get("pitch", 0.0))
            yaw_deg = math.degrees(data.get("yaw", 0.0))
            return f"Roll: {roll_deg:6.1f}° | Pitch: {pitch_deg:6.1f}° | Yaw: {yaw_deg:6.1f}°"

        elif pkt_type == "VFR_HUD":
            return (
                f"Airspeed: {data.get('airspeed', 0.0):4.1f} m/s | "
                f"GndSpeed: {data.get('groundspeed', 0.0):4.1f} m/s | "
                f"Heading: {data.get('heading', 0):3d}° | "
                f"Alt: {data.get('alt', 0.0):5.2f}m"
            )

        elif pkt_type == "SYS_STATUS":
            voltage = data.get("voltage_battery", 0) / 1000.0
            current = data.get("current_battery", 0) / 100.0
            rem = data.get("battery_remaining", -1)
            cpu_load = data.get("load", 0) / 10.0
            return f"Battery: {voltage:5.2f}V ({current:4.1f}A, {rem}%) | CPU Load: {cpu_load:4.1f}%"

        elif pkt_type == "GLOBAL_POSITION_INT":
            lat = data.get("lat", 0) / 1e7
            lon = data.get("lon", 0) / 1e7
            rel_alt = data.get("relative_alt", 0) / 1000.0
            vx = data.get("vx", 0) / 100.0
            vy = data.get("vy", 0) / 100.0
            vz = data.get("vz", 0) / 100.0
            return f"Pos: [{lat:.5f}, {lon:.5f}] | RelAlt: {rel_alt:5.2f}m | Vel: [{vx:4.1f}, {vy:4.1f}, {vz:4.1f}] m/s"

        elif pkt_type == "POWER_STATUS":
            vcc = data.get("Vcc", 0) / 1000.0
            vservo = data.get("Vservo", 0) / 1000.0
            return f"Vcc: {vcc:4.2f}V | Vservo: {vservo:4.2f}V"

        elif pkt_type == "NAV_CONTROLLER_OUTPUT":
            return (
                f"NavRoll: {data.get('nav_roll', 0.0):5.2f}° | "
                f"NavPitch: {data.get('nav_pitch', 0.0):5.2f}° | "
                f"WP Dist: {data.get('wp_dist', 0)}m"
            )

        # Fallback for generic or unformatted custom telemetry packets
        clean_fields = {k: v for k, v in data.items() if k not in ["mavpackettype", "time_boot_ms", "time_usec"]}
        return str(clean_fields)

    def get_velocity(self) -> Dict[str, float]:
        """Extracts 3D velocities in m/s from the latest cached GLOBAL_POSITION_INT packet."""
        data = self.latest_telemetry_cache.get("GLOBAL_POSITION_INT")
        if data:
            return {
                "vx": round(data.get("vx", 0) / 100.0, 3),
                "vy": round(data.get("vy", 0) / 100.0, 3),
                "vz": round(data.get("vz", 0) / 100.0, 3),
            }
        return {"vx": 0.0, "vy": 0.0, "vz": 0.0}