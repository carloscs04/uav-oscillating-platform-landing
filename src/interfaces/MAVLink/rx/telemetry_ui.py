import math
import time
from typing import Any, Dict
from pymavlink import mavutil
from src.interfaces.MAVLink.state.vehicle_state import VehicleState


class TelemetryUI:
    """Formats and prints low-frequency telemetry summaries to terminal console."""

    ACK_RESULTS = {
        0: "ACCEPTED 🟢",
        1: "TEMPORARILY_REJECTED 🟡",
        2: "DENIED 🔴",
        3: "UNSUPPORTED ❌",
        4: "FAILED ❌",
        5: "IN_PROGRESS ⏳"
    }

    def __init__(self, state: VehicleState):
        self.state = state

    def format_packet_summary(self, pkt_type: str, data: Dict[str, Any]) -> str:
        if pkt_type == "HEARTBEAT":
            base_mode = data.get("base_mode", 0)
            custom_mode = data.get("custom_mode", 0)
            is_armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            arm_str = "ARMED 🟢" if is_armed else "DISARMED 🔴"
            return f"Status: {arm_str} | BaseMode: {base_mode} | CustomMode: {custom_mode}"

        elif pkt_type == "STATUSTEXT":
            text = data.get("text", "")
            # Clean bytes or byte arrays returned by pymavlink
            if isinstance(text, (bytes, bytearray)):
                text = text.decode("utf-8", errors="ignore").rstrip("\x00")
            elif isinstance(text, list):
                text = "".join([chr(c) for c in text if c != 0])
            severity = data.get("severity", 0)
            return f"Msg: '{text.strip()}' (Severity: {severity})"

        elif pkt_type == "COMMAND_ACK":
            cmd_id = data.get("command", 0)
            result_code = data.get("result", 4)
            result_str = self.ACK_RESULTS.get(result_code, f"RESULT_{result_code}")
            return f"Command ID {cmd_id}: {result_str}"

        elif pkt_type == "ATTITUDE":
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

        clean_fields = {k: v for k, v in data.items() if k not in ["mavpackettype", "time_boot_ms", "time_usec"]}
        return str(clean_fields)

    def print_summary(self, display_interval: float = 1.0):
        watched = self.state.get_watched_packets()
        t_stamp = time.strftime("%H:%M:%S")
        print(f"\n--- [TELEMETRY SUMMARY @ {t_stamp}] (Refresh Rate: {display_interval}s) ---")
        
        for pkt_type in watched:
            raw_data = self.state.get_packet(pkt_type)
            if raw_data:
                formatted_str = self.format_packet_summary(pkt_type, raw_data)
                print(f" > [{pkt_type}]: {formatted_str}")
            else:
                print(f" > [{pkt_type}]: Awaiting packet stream...")