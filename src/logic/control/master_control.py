# File: src/logic/control/master_control.py

import threading
import time
from enum import Enum
from typing import Any, Optional, Tuple
from src.logic.control.control_state import ControlState


class MissionPhase(str, Enum):
    IDLE = "IDLE"
    ARMING_AND_TAKEOFF = "ARMING_AND_TAKEOFF"
    HOVER = "HOVER"
    ALIGNING = "ALIGNING"
    LANDING = "LANDING"


class ControlManager:
    """Facade API & Mission Controller for 3-Axis Precision Landing."""

    def __init__(
        self,
        mavlink_node: Optional[Any] = None,
        vision_node: Optional[Any] = None,
        distance_node: Optional[Any] = None,
        target_hz: float = 20.0,
    ):
        self.mavlink = mavlink_node
        self.vision = vision_node
        self.distance = distance_node

        self.target_hz = target_hz
        self.active_guidance: bool = False
        self.phase: MissionPhase = MissionPhase.IDLE
        self.target_altitude_m: float = 1.5
        self.setpoint: Tuple[float, float, float] = (0.0, 0.0, 1.5)

        self.state = ControlState()
        self.connected: bool = False
        self.running: bool = False
        self._thread: Optional[threading.Thread] = None

    def bind_subsystems(self, mavlink_node: Any, vision_node: Any = None, distance_node: Any = None):
        self.mavlink = mavlink_node
        self.vision = vision_node
        self.distance = distance_node

    def start(self) -> bool:
        if self.running:
            return self.connected
        self.running = True
        self.connected = True
        setattr(self.state, "connected", True)
        setattr(self.state, "current_rate_hz", self.target_hz)

        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self.active_guidance = False
        self.phase = MissionPhase.IDLE
        self.running = False
        self.connected = False
        setattr(self.state, "connected", False)
        setattr(self.state, "current_rate_hz", 0.0)

    def is_healthy(self) -> bool:
        return self.running and self.connected

    def start_mission(self, altitude_m: float = 1.5) -> bool:
        if not self.mavlink:
            return False
        self.target_altitude_m = float(altitude_m)
        self.setpoint = (0.0, 0.0, self.target_altitude_m)
        self.active_guidance = True
        self.phase = MissionPhase.ARMING_AND_TAKEOFF
        return True

    def hold_hover(self, force_sim: bool = False) -> bool:
        if not self.mavlink:
            return False
        if not force_sim and not self.mavlink.state.is_armed():
            return False

        self.phase = MissionPhase.HOVER
        if self.mavlink.is_healthy():
            self.mavlink.send_velocity(0.0, 0.0, 0.0)
        return True

    def abort_mission(self):
        self.active_guidance = False
        self.phase = MissionPhase.IDLE

    def enable_guidance(self):
        self.active_guidance = True

    def disable_guidance(self):
        self.active_guidance = False
        self.phase = MissionPhase.IDLE

    def set_target_offset(self, ref_x: float, ref_y: float, ref_z: float):
        self.setpoint = (ref_x, ref_y, ref_z)

    def _control_loop(self):
        interval = 1.0 / max(1.0, self.target_hz)
        while self.running:
            if not self.active_guidance or not self.mavlink:
                time.sleep(interval)
                continue

            if self.phase == MissionPhase.ARMING_AND_TAKEOFF:
                self._execute_arm_and_takeoff()
            elif self.phase == MissionPhase.HOVER:
                pass
            elif self.phase == MissionPhase.ALIGNING:
                pass
            elif self.phase == MissionPhase.LANDING:
                pass

            time.sleep(interval)

    def _execute_arm_and_takeoff(self):
        self.mavlink.set_mode("GUIDED")
        time.sleep(0.3)

        if not self.mavlink.state.is_armed():
            self.mavlink.arm()
            time.sleep(0.8)

        if not self.mavlink.state.is_armed():
            self.active_guidance = False
            self.phase = MissionPhase.IDLE
            return

        self.mavlink.takeoff(self.target_altitude_m)
        time.sleep(0.5)
        self.phase = MissionPhase.HOVER


MasterControl = ControlManager