import time
from typing import Any, Optional
from pymavlink import mavutil


class SystemCommands:
    """Handles system lifecycle commands, mode changes, stream setup, and debug overrides."""

    def __init__(self):
        self.master: Optional[Any] = None

    def set_master(self, master: Any):
        self.master = master

    def request_telemetry_streams(self, rate_hz: int = 10):
        if not self.master:
            return
        try:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                rate_hz,
                1
            )
            print(f"[System TX] Telemetry stream requested at {rate_hz}Hz.")
        except Exception as e:
            print(f"[System TX Error] Failed requesting telemetry streams: {e}")

    def stop_telemetry_streams(self):
        if not self.master:
            return
        try:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                0,
                0
            )
            print("[System TX] Telemetry stream stopped.")
        except Exception as e:
            print(f"[System TX Error] Failed stopping telemetry streams: {e}")

    def arm(self) -> bool:
        """Clean standard arm command (respects ArduPilot safety checks)."""
        if not self.master:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            print("[System TX] Standard 'ARM' command sent.")
            return True
        except Exception as e:
            print(f"[System TX Error] Failed sending ARM command: {e}")
            return False

    def disarm(self) -> bool:
        """Clean standard disarm command."""
        if not self.master:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            print("[System TX] Standard 'DISARM' command sent.")
            return True
        except Exception as e:
            print(f"[System TX Error] Failed sending DISARM command: {e}")
            return False

    def set_mode(self, mode_name: str) -> bool:
        if not self.master:
            return False
        mode_mapping = self.master.mode_mapping()
        if not mode_mapping or mode_name.upper() not in mode_mapping:
            print(f"[System TX Error] Mode '{mode_name}' unsupported.")
            return False
        try:
            self.master.set_mode(mode_mapping[mode_name.upper()])
            print(f"[System TX] Mode changed to: {mode_name.upper()}")
            return True
        except Exception as e:
            print(f"[System TX Error] Failed setting mode: {e}")
            return False

    def debug_force_arm(self) -> bool:
        """Force arms motors for bench testing with delays to ensure params land."""
        if not self.master:
            return False
        try:
            params = [
                (b"ARMING_CHECK", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                (b"DISARM_DELAY", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                (b"BRD_SAFETYENABLE", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ]
            for name, val, ptype in params:
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    name, val, ptype
                )
                time.sleep(0.05)  # 50ms buffer to allow Pixhawk EEPROM/RAM to process

            # Send arm command
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0, 21196.0, 0, 0, 0, 0, 0
            )
            print("[System TX Debug] Parameter overrides sent & FORCE ARM executed.")
            return True
        except Exception as e:
            print(f"[System TX Error] Failed force arming: {e}")
            return False

    def debug_force_arm(self) -> bool:
        """Force arms motors for bench testing, overriding safety checks & radio failsafe."""
        if not self.master:
            return False
        try:
            params = [
                (b"ARMING_CHECK", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                (b"DISARM_DELAY", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                (b"BRD_SAFETYENABLE", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                (b"FS_THR_ENABLE", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),  # Disable Radio Failsafe for bench test
            ]
            for name, val, ptype in params:
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    name, val, ptype
                )
                time.sleep(0.05)

            # Send force arm command
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0, 21196.0, 0, 0, 0, 0, 0
            )
            print("[System TX Debug] Parameter overrides sent & FORCE ARM executed.")
            return True
        except Exception as e:
            print(f"[System TX Error] Failed force arming: {e}")
            return False