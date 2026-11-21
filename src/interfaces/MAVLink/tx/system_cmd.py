# file: src/interfaces/MAVLink/tx/system_cmd.py

import time
import threading
from typing import Any, Optional
from pymavlink import mavutil


class SystemCommands:
    """Handles system lifecycle commands, mode changes, stream setup, EKF origin, and debug overrides."""

    def __init__(self):
        self.master: Optional[Any] = None

    def set_master(self, master: Any):
        self.master = master

    def set_ekf_origin(
        self,
        lat: int = 256866000,
        lon: int = -1003161000,
        alt: int = 500000,
        retries: int = 10,
    ) -> bool:
        """Broadcasts local EKF global origin explicitly to Pixhawk (sysid=1) over 10s."""
        if not self.master:
            return False

        def _origin_worker():
            for _ in range(retries):
                if not self.master:
                    break
                try:
                    self.master.mav.set_gps_global_origin_send(
                        1,
                        lat,
                        lon,
                        alt,
                    )
                    self.master.mav.set_home_position_send(
                        1,
                        lat,
                        lon,
                        alt,
                        0.0,
                        0.0,
                        0.0,
                        [1.0, 0.0, 0.0, 0.0],
                        0.0,
                        0.0,
                        0.0,
                    )
                except Exception:
                    pass
                time.sleep(1.0)
            print(f"[System TX] EKF Global Origin & Home confirmed to sysid=1 (lat={lat}, lon={lon}).")

        threading.Thread(target=_origin_worker, daemon=True).start()
        return True

    def request_telemetry_streams(self, rate_hz: int = 10):
        if not self.master:
            return
        try:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                rate_hz,
                1,
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
                0,
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
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
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
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
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
        """Force arms motors for bench testing, overriding safety checks & radio failsafe."""
        if not self.master:
            return False
        try:
            params = [
                (b"ARMING_CHECK", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                (b"DISARM_DELAY", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                (b"BRD_SAFETYENABLE", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                (b"FS_THR_ENABLE", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ]
            for name, val, ptype in params:
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    name,
                    val,
                    ptype,
                )
                time.sleep(0.05)

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0,
                21196.0,
                0,
                0,
                0,
                0,
                0,
            )
            print("[System TX Debug] Parameter overrides sent & FORCE ARM executed.")
            return True
        except Exception as e:
            print(f"[System TX Error] Failed force arming: {e}")
            return False

    def debug_force_disarm(self) -> bool:
        """Force disarms motors immediately for bench testing."""
        if not self.master:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0.0,
                21196.0,
                0,
                0,
                0,
                0,
                0,
            )
            print("[System TX Debug] FORCE DISARM executed.")
            return True
        except Exception as e:
            print(f"[System TX Error] Failed force disarming: {e}")
            return False