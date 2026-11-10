from typing import Any, Optional
from pymavlink import mavutil


class FlightCommands:
    """Dispatches movement targets, velocity setpoints, takeoff, and landing."""

    def __init__(self):
        self.master: Optional[Any] = None

    def set_master(self, master: Any):
        self.master = master

    def send_velocity(self, vx: float, vy: float, vz: float):
        """Sends Local NED velocity target setpoints (m/s)."""
        if not self.master:
            return
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # Ignore position, use velocity only
            0.0, 0.0, 0.0,
            vx, vy, vz,
            0.0, 0.0, 0.0,
            0.0, 0.0
        )

    def takeoff(self, altitude_m: float = 1.5) -> bool:
        """Sends takeoff command to specified target altitude in meters."""
        if not self.master:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, altitude_m
            )
            print(f"[Flight TX] 'TAKEOFF' command ({altitude_m}m) sent.")
            return True
        except Exception as e:
            print(f"[Flight TX Error] Failed sending TAKEOFF command: {e}")
            return False

    def land(self) -> bool:
        """Triggers landing by changing mode to LAND."""
        if not self.master:
            return False
        mode_mapping = self.master.mode_mapping()
        if not mode_mapping or "LAND" not in mode_mapping:
            return False
        try:
            self.master.set_mode(mode_mapping["LAND"])
            print("[Flight TX] 'LAND' command sent.")
            return True
        except Exception as e:
            print(f"[Flight TX Error] Failed sending LAND command: {e}")
            return False