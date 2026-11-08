# src/interfaces/MAVLink/tx_cmd/driver_tx.py

from typing import Any, Optional
from pymavlink import mavutil


class MAVLinkTXDriver:
    """MAVLink Transmitter Driver: Exclusively handles sending commands, setpoints, 
    and stream controls to the Pixhack flight controller.
    """

    def __init__(self, master: Optional[Any] = None):
        self.master = master

    def set_master(self, master: Any):
        """Assigns the shared MAVLink hardware connection instance."""
        self.master = master

    def request_telemetry_streams(self, rate_hz: int = 10):
        """Requests the flight controller to start continuous telemetry stream output."""
        if not self.master:
            return
        try:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                rate_hz,
                1  # 1 = Start stream
            )
            print(f"[TX Driver] MAVLink stream request sent at {rate_hz}Hz.")
        except Exception as e:
            print(f"[TX Driver Error] Failed to request streams: {e}")

    def stop_telemetry_streams(self):
        """Requests the flight controller to pause telemetry output (turns off hardware TX LED)."""
        if not self.master:
            return
        try:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                0,  # 0Hz rate
                0   # 0 = Stop stream
            )
            print("[TX Driver] Stream shutdown command sent to flight controller.")
        except Exception as e:
            print(f"[TX Driver Error] Failed to stop streams: {e}")

    def send_velocity(self, vx: float, vy: float, vz: float):
        """Sends Local NED velocity target setpoints."""
        if not self.master:
            return
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0.0, 0.0, 0.0,
            vx, vy, vz,
            0.0, 0.0, 0.0,
            0.0, 0.0
        )

    def arm(self) -> bool:
        """Sends command to arm flight controller motors."""
        if not self.master:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            print("[TX Driver] 'ARM' command sent.")
            return True
        except Exception as e:
            print(f"[TX Driver Error] Failed to send ARM command: {e}")
            return False

    def disable_arming_checks(self) -> bool:
        """Disables all Pre-Arm checks and auto-disarm timeouts for bench testing."""
        if not self.master:
            return False
        try:
            # 1. Desactiva todas las comprobaciones PreArm (0.0 = Desactivado)
            self.master.param_set_send(
                'ARMING_CHECK',
                0.0,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            # 2. Desactiva el temporizador de auto-desarmado por inactividad
            self.master.param_set_send(
                'DISARM_DELAY',
                0.0,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            print("[TX Driver] Sent parameter overrides: ARMING_CHECK=0, DISARM_DELAY=0.")
            return True
        except Exception as e:
            print(f"[TX Driver Error] Failed to disable arming checks: {e}")
            return False

    def force_arm(self) -> bool:
        if not self.master:
            return False
        try:
            # Desactivar chequeos, autodesarmado y el interruptor físico de seguridad
            params = [
                (b"ARMING_CHECK", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                (b"DISARM_DELAY", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
                (b"BRD_SAFETYENABLE", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            ]
            
            for name, val, ptype in params:
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    name,
                    val,
                    ptype
                )
            
            # Enviar comando de armado
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0, 21196.0, 0, 0, 0, 0, 0
            )
            print("[TX Driver] Sent parameter overrides & FORCE ARM command.")
            return True
        except Exception as e:
            print(f"[TX Driver Error] Failed to force arm: {e}")
            return False

    def send_rc_override(self, pitch=1500, roll=1500, throttle=1000, yaw=1500):
        """Sends dummy RC channel signals to clear Radio Failsafe on bench test."""
        if not self.master:
            return
        try:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                pitch,     # Ch1
                roll,      # Ch2
                throttle,  # Ch3
                yaw,       # Ch4
                0, 0, 0, 0 # Ch5-8 deshabilitados
            )
        except Exception as e:
            print(f"[TX Driver Error] Failed to send RC override: {e}")

    def disarm(self) -> bool:
        """Sends command to disarm flight controller motors."""
        if not self.master:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            print("[TX Driver] 'DISARM' command sent.")
            return True
        except Exception as e:
            print(f"[TX Driver Error] Failed to send DISARM command: {e}")
            return False

    def force_disarm(self) -> bool:
        """Forces motor disarming, bypassing safety and throttle checks."""
        if not self.master:
            return False
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0.0,      # 0.0 = Disarm
                21196.0,  # Magic number 21196 forces disarming
                0, 0, 0, 0, 0
            )
            print("[TX Driver] 'FORCE DISARM' command sent.")
            return True
        except Exception as e:
            print(f"[TX Driver Error] Failed to force disarm: {e}")
            return False

    def set_mode(self, mode_name: str) -> bool:
        """Changes flight mode (e.g., 'GUIDED', 'LOITER', 'ALT_HOLD', 'LAND')."""
        if not self.master:
            return False
        mode_mapping = self.master.mode_mapping()
        if not mode_mapping or mode_name.upper() not in mode_mapping:
            print(f"[TX Driver Error] Mode '{mode_name}' is not supported by flight controller.")
            return False
        try:
            self.master.set_mode(mode_mapping[mode_name.upper()])
            print(f"[TX Driver] Flight mode changed to: {mode_name.upper()}")
            return True
        except Exception as e:
            print(f"[TX Driver Error] Failed to change flight mode: {e}")
            return False

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
            print(f"[TX Driver] 'TAKEOFF' command ({altitude_m}m) sent.")
            return True
        except Exception as e:
            print(f"[TX Driver Error] Failed to send TAKEOFF command: {e}")
            return False