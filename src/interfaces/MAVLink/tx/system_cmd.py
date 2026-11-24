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

    # msg_id -> Hz. REQUEST_DATA_STREAM(ALL) alone left VIBRATION all-zero in the flight
    # CSVs, so ask for the messages the flow/Loiter tuning needs one by one as well.
    MESSAGE_INTERVALS_HZ = {
        241: 10,  # VIBRATION
        193: 10,  # EKF_STATUS_REPORT
        100: 20,  # OPTICAL_FLOW (ArduPilot's own view of the flow sensor)
        132: 20,  # DISTANCE_SENSOR
        32: 20,   # LOCAL_POSITION_NED
        30: 25,   # ATTITUDE
        62: 10,   # NAV_CONTROLLER_OUTPUT
        36: 10,   # SERVO_OUTPUT_RAW
        1: 2,     # SYS_STATUS (sensor health bits, used by 'prearm')
        29: 10,   # SCALED_PRESSURE (baro altitude column in the ground station CSV)
    }

    def request_message_intervals(self):
        if not self.master:
            return
        for msg_id, hz in self.MESSAGE_INTERVALS_HZ.items():
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    msg_id,
                    int(1_000_000 / hz),
                    0, 0, 0, 0, 0,
                )
                time.sleep(0.02)
            except Exception as e:
                print(f"[System TX Error] SET_MESSAGE_INTERVAL {msg_id}: {e}")
                return
        print(f"[System TX] Message intervals requested for {len(self.MESSAGE_INTERVALS_HZ)} messages.")

    def run_prearm_checks(self) -> bool:
        """Ask ArduPilot to run its pre-arm checks now; it answers with 'PreArm: ...' texts."""
        if not self.master:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                401,  # MAV_CMD_RUN_PREARM_CHECKS
                0, 0, 0, 0, 0, 0, 0, 0,
            )
            return True
        except Exception as e:
            print(f"[System TX Error] RUN_PREARM_CHECKS: {e}")
            return False

    # MAV_CMD_PREFLIGHT_CALIBRATION param index (1-based) and value for each calibration
    CALIBRATIONS = {
        "gyro": (1, 1),    # gyro offsets (keep the vehicle perfectly still)
        "baro": (3, 1),    # ground pressure = 0 m (needed before using the baro as height)
        "level": (5, 2),   # board level ("level horizon", trims AHRS_TRIM_X/Y)
    }

    def preflight_calibration(self, kind: str) -> bool:
        if not self.master or kind not in self.CALIBRATIONS:
            return False
        idx, val = self.CALIBRATIONS[kind]
        params = [0.0] * 7
        params[idx - 1] = float(val)
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                *params,
            )
            return True
        except Exception as e:
            print(f"[System TX Error] PREFLIGHT_CALIBRATION {kind}: {e}")
            return False

    def set_ekf_source_set(self, source_set: int) -> bool:
        """MAV_CMD_SET_EKF_SOURCE_SET (42007), sent as COMMAND_INT as ArduPilot handles it.
        1 = EK3_SRC1_* (lidar height), 2 = EK3_SRC2_* (baro height)."""
        if not self.master or source_set not in (1, 2, 3):
            return False
        try:
            self.master.mav.command_int_send(
                self.master.target_system,
                self.master.target_component,
                0, 42007, 0, 0,
                float(source_set), 0, 0, 0, 0, 0, 0,
            )
            return True
        except Exception as e:
            print(f"[System TX Error] SET_EKF_SOURCE_SET {source_set}: {e}")
            return False

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