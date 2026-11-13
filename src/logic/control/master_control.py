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
    """Facade API & Unified Flight Controller (Basic Takeoff/Hover + Autonomous ArUco Mission)."""

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
        self.mission_mode: bool = False  # False = basic takeoff/hover only | True = full autonomous ArUco mission
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
        self.mission_mode = False
        self.phase = MissionPhase.IDLE
        self.running = False
        self.connected = False
        setattr(self.state, "connected", False)
        setattr(self.state, "current_rate_hz", 0.0)

    def is_healthy(self) -> bool:
        return self.running and self.connected

    def is_airborne(self) -> bool:
        """Returns True whenever ControlManager is actively executing an airborne phase."""
        return self.active_guidance and self.phase in {
            MissionPhase.ARMING_AND_TAKEOFF,
            MissionPhase.HOVER,
            MissionPhase.ALIGNING,
            MissionPhase.LANDING,
        }

    # =========================================================================
    # BASIC FLIGHT BEHAVIORS (TAKEOFF / HOVER / LAND WITHOUT MISSION ALIGNMENT)
    # =========================================================================

    def start_basic_flight(self, altitude_m: float = 1.5) -> bool:
        """Arms (if needed), takes off to altitude_m, and holds HOVER indefinitely without mission alignment."""
        if not self.mavlink:
            return False

        self.mission_mode = False
        self.target_altitude_m = float(altitude_m)
        self.setpoint = (0.0, 0.0, self.target_altitude_m)
        self.active_guidance = True

        if self.mavlink.state.is_armed() and self.phase in {MissionPhase.HOVER, MissionPhase.ALIGNING}:
            self.phase = MissionPhase.HOVER
        else:
            self.phase = MissionPhase.ARMING_AND_TAKEOFF
        return True

    def hold_hover(self, force_sim: bool = False) -> bool:
        """Cancels alignment/motion and keeps the vehicle holding HOVER."""
        if not self.mavlink:
            return False
        if not force_sim and not self.mavlink.state.is_armed():
            return False

        self.mission_mode = False
        self.active_guidance = True
        self.phase = MissionPhase.HOVER
        if self.mavlink.is_healthy():
            self.mavlink.send_velocity(0.0, 0.0, 0.0)
        return True

    def command_land(self) -> bool:
        """Stops active guidance loop and commands Pixhawk to LAND."""
        if not self.mavlink:
            return False

        self.mission_mode = False
        self.active_guidance = False
        self.phase = MissionPhase.IDLE
        if self.mavlink.is_healthy():
            return self.mavlink.land()
        return False

    # =========================================================================
    # FULL AUTONOMOUS MISSION (WORKS FROM GROUND OR MID-AIR HOVER)
    # =========================================================================

    def start_mission(self, altitude_m: float = 1.5) -> bool:
        """
        Starts full autonomous ArUco mission regardless of initial state:
        - If on ground (disarmed): Arms -> Takeoffs -> Hovers -> Aligns ArUco -> Precision Lands.
        - If already airborne (armed/hovering): Skips takeoff and jumps directly to Aligning ArUco -> Precision Lands.
        """
        if not self.mavlink:
            return False

        self.mission_mode = True
        self.target_altitude_m = float(altitude_m)
        self.setpoint = (0.0, 0.0, self.target_altitude_m)
        self.active_guidance = True

        if self.mavlink.state.is_armed():
            self.phase = MissionPhase.ALIGNING
        else:
            self.phase = MissionPhase.ARMING_AND_TAKEOFF
        return True

    def abort_mission(self):
        self.mission_mode = False
        self.active_guidance = False
        self.phase = MissionPhase.IDLE

    def enable_guidance(self):
        self.active_guidance = True

    def disable_guidance(self):
        self.mission_mode = False
        self.active_guidance = False
        self.phase = MissionPhase.IDLE

    def set_target_offset(self, ref_x: float, ref_y: float, ref_z: float):
        self.setpoint = (ref_x, ref_y, ref_z)

    # =========================================================================
    # 20 HZ CLOSED-LOOP EXECUTION ENGINE
    # =========================================================================

    def _control_loop(self):
        interval = 1.0 / max(1.0, self.target_hz)
        while self.running:
            if not self.active_guidance or not self.mavlink:
                time.sleep(interval)
                continue

            # Safety watchdog: immediately halt guidance if MAVLink drops mid-flight
            if not self.mavlink.is_healthy():
                self.mission_mode = False
                self.active_guidance = False
                self.phase = MissionPhase.IDLE
                time.sleep(interval)
                continue

            if self.phase == MissionPhase.ARMING_AND_TAKEOFF:
                self._execute_arm_and_takeoff()

            elif self.phase == MissionPhase.HOVER:
                self.mavlink.send_velocity(0.0, 0.0, 0.0)
                if self.mission_mode:
                    self.phase = MissionPhase.ALIGNING

            elif self.phase == MissionPhase.ALIGNING:
                # TODO: Read ArUco offset from self.vision and send clamped vx, vy setpoints
                # Once centered within threshold, transition to self.phase = MissionPhase.LANDING
                pass

            elif self.phase == MissionPhase.LANDING:
                # TODO: Descend while keeping ArUco centered, then call self.command_land()
                pass

            time.sleep(interval)

    def _execute_arm_and_takeoff(self):
        self.mavlink.set_mode("GUIDED_NOGPS")
        time.sleep(0.3)

        if not self.mavlink.state.is_armed():
            self.mavlink.arm()
            time.sleep(0.8)

        if not self.mavlink.state.is_armed():
            self.mission_mode = False
            self.active_guidance = False
            self.phase = MissionPhase.IDLE
            return

        self.mavlink.takeoff(self.target_altitude_m)
        time.sleep(0.5)
        self.phase = MissionPhase.HOVER


MasterControl = ControlManager