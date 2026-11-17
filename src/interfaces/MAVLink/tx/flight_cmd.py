# File: src/interfaces/MAVLink/tx/flight_cmd.py

import math
from typing import Any, Optional
from pymavlink import mavutil


class FlightCommands:
    """Dispatches movement targets, attitude/thrust setpoints, velocity setpoints, takeoff, and landing."""

    def __init__(self):
        self.master: Optional[Any] = None

    def set_master(self, master: Any):
        self.master = master

    @staticmethod
    def _euler_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [w, x, y, z]

    def send_attitude_target(
        self,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        yaw_deg: float = 0.0,
        thrust: float = 0.5,
    ):
        """
        Sends SET_ATTITUDE_TARGET for GUIDED_NOGPS flight.
        thrust: 0.0 to 1.0 (0.5 = hover, >0.5 = climb, <0.5 = descend).
        """
        if not self.master:
            return
        thrust_clamped = max(0.0, min(1.0, float(thrust)))
        q = self._euler_to_quaternion(roll_deg, pitch_deg, yaw_deg)
        # type_mask = 0b00000111 (7) -> ignore body roll/pitch/yaw rates, use attitude quaternion + thrust
        self.master.mav.set_attitude_target_send(
            0,
            self.master.target_system,
            self.master.target_component,
            0b00000111,
            q,
            0.0,
            0.0,
            0.0,
            thrust_clamped,
        )

    def send_velocity(self, vx: float, vy: float, vz: float):
        """Sends Local NED velocity target setpoints (m/s)."""
        if not self.master:
            return
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # Ignore position, use velocity only
            0.0,
            0.0,
            0.0,
            vx,
            vy,
            vz,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def takeoff(self, altitude_m: float = 1.5) -> bool:
        """Sends standard NAV_TAKEOFF command (used when GPS/EKF position is available)."""
        if not self.master:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                altitude_m,
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