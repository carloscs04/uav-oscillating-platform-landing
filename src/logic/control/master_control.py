# File: src/logic/control/master_control.py

import threading
import time
from enum import Enum
from typing import Any, Optional, Tuple
from src.logic.control.control_state import ControlState


class MissionPhase(str, Enum):
    IDLE = "IDLE"
    ARMING = "ARMING"
    CLIMBING = "CLIMBING"
    HOVER = "HOVER"
    ALIGNING = "ALIGNING"
    LANDING = "LANDING"


class ControlManager:
    """Facade API & Unified Flight Controller (GUIDED_NOGPS Continuous Z-PID + ArUco Mission)."""

    HOVER_THRUST = 0.50
    FALLBACK_CLIMB_THRUST = 0.56
    MIN_THRUST = 0.30
    MAX_THRUST = 0.72

    # Continuous Z-axis PID gains (active from takeoff through hover/aligning)
    ALT_KP = 0.22
    ALT_KI = 0.05
    ALT_KD = 0.14
    ALT_I_MAX = 0.14

    # Reference stabilization criteria to transition status CLIMBING -> HOVER
    ALT_TOLERANCE_M = 0.06       # +/- 6 cm around target altitude
    VZ_STABLE_THRESH = 0.15      # m/s vertical speed threshold
    STABLE_HOLD_SEC = 0.60       # must remain within tolerance for 0.6s

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
        self.mission_mode: bool = False
        self.phase: MissionPhase = MissionPhase.IDLE
        self.target_altitude_m: float = 1.5
        self.setpoint: Tuple[float, float, float] = (0.0, 0.0, 1.5)

        # Continuous Z-PID controller state
        self._alt_integral: float = 0.0
        self._prev_alt_m: Optional[float] = None
        self._prev_alt_time: Optional[float] = None
        self._filtered_vz: float = 0.0
        self._stable_since: Optional[float] = None
        self.last_thrust: float = self.HOVER_THRUST

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
        self._reset_altitude_pid()
        setattr(self.state, "connected", False)
        setattr(self.state, "current_rate_hz", 0.0)

    def is_healthy(self) -> bool:
        return self.running and self.connected

    def is_airborne(self) -> bool:
        """Returns True whenever ControlManager is actively executing an airborne phase."""
        return self.active_guidance and self.phase in {
            MissionPhase.ARMING,
            MissionPhase.CLIMBING,
            MissionPhase.HOVER,
            MissionPhase.ALIGNING,
            MissionPhase.LANDING,
        }

    # =========================================================================
    # BASIC FLIGHT BEHAVIORS (TAKEOFF / HOVER / LAND WITHOUT MISSION ALIGNMENT)
    # =========================================================================

    def start_basic_flight(self, altitude_m: float = 1.5) -> bool:
        """Arms (if needed) and runs continuous Z-PID until stabilized at target altitude."""
        if not self.mavlink:
            return False

        self.mission_mode = False
        self.target_altitude_m = float(altitude_m)
        self.setpoint = (0.0, 0.0, self.target_altitude_m)
        self._stable_since = None
        self.active_guidance = True

        if self.mavlink.state.is_armed() and self.phase in {MissionPhase.CLIMBING, MissionPhase.HOVER, MissionPhase.ALIGNING}:
            self.phase = MissionPhase.CLIMBING
        else:
            self._reset_altitude_pid(initial_alt=self._get_current_altitude_m())
            self.phase = MissionPhase.ARMING
        return True

    def hold_hover(self, force_sim: bool = False) -> bool:
        """Cancels alignment and keeps holding current/target altitude with the Z-PID."""
        if not self.mavlink:
            return False
        if not force_sim and not self.mavlink.state.is_armed():
            return False

        cur_alt = self._get_current_altitude_m()
        if cur_alt is not None and self.phase == MissionPhase.CLIMBING:
            self.target_altitude_m = cur_alt
            self.setpoint = (self.setpoint[0], self.setpoint[1], cur_alt)

        self._stable_since = None
        self.mission_mode = False
        self.active_guidance = True
        self.phase = MissionPhase.HOVER

        if self.mavlink.is_healthy():
            thrust, _ = self._compute_altitude_thrust(time.monotonic())
            self.mavlink.send_attitude_target(0.0, 0.0, 0.0, thrust)
        return True

    def command_land(self) -> bool:
        """Stops active guidance loop and commands Pixhawk to LAND."""
        if not self.mavlink:
            return False

        self.mission_mode = False
        self.active_guidance = False
        self.phase = MissionPhase.IDLE
        self._reset_altitude_pid()
        if self.mavlink.is_healthy():
            return self.mavlink.land()
        return False

    # =========================================================================
    # FULL AUTONOMOUS MISSION (WORKS FROM GROUND OR MID-AIR)
    # =========================================================================

    def start_mission(self, altitude_m: float = 1.5) -> bool:
        """
        Starts full autonomous ArUco mission:
        - If disarmed: Arms -> Climbs via Z-PID until stabilized -> Aligns ArUco -> Lands.
        - If already armed: Climbs/holds via Z-PID and starts Aligning ArUco -> Lands.
        """
        if not self.mavlink:
            return False

        self.mission_mode = True
        self.target_altitude_m = float(altitude_m)
        self.setpoint = (0.0, 0.0, self.target_altitude_m)
        self._stable_since = None
        self.active_guidance = True

        if self.mavlink.state.is_armed():
            self.phase = MissionPhase.ALIGNING
        else:
            self._reset_altitude_pid(initial_alt=self._get_current_altitude_m())
            self.phase = MissionPhase.ARMING
        return True

    def abort_mission(self):
        self.mission_mode = False
        self.active_guidance = False
        self.phase = MissionPhase.IDLE
        self._reset_altitude_pid()

    def enable_guidance(self):
        self.active_guidance = True

    def disable_guidance(self):
        self.mission_mode = False
        self.active_guidance = False
        self.phase = MissionPhase.IDLE
        self._reset_altitude_pid()

    def set_target_offset(self, ref_x: float, ref_y: float, ref_z: float):
        old_z = self.target_altitude_m
        self.setpoint = (float(ref_x), float(ref_y), float(ref_z))
        if ref_z > 0.05:
            self.target_altitude_m = float(ref_z)
            if abs(self.target_altitude_m - old_z) > self.ALT_TOLERANCE_M:
                self._stable_since = None
                if self.phase == MissionPhase.HOVER:
                    self.phase = MissionPhase.CLIMBING

    # =========================================================================
    # ALTITUDE SENSOR & CONTINUOUS Z-AXIS PID CONTROLLER
    # =========================================================================

    def _get_current_altitude_m(self) -> Optional[float]:
        """Reads downward VL53L1X distance sensor if healthy and reporting a valid target."""
        if not self.distance:
            return None

        if hasattr(self.distance, "is_healthy") and not self.distance.is_healthy():
            return None

        state = getattr(self.distance, "state", self.distance)
        if getattr(state, "sensor_status", "OK") != "OK":
            return None

        dist_m = getattr(state, "distance_m", None)
        min_m = float(getattr(state, "min_distance_m", 0.04))
        max_m = float(getattr(state, "max_distance_m", 4.0))

        if isinstance(dist_m, (int, float)) and min_m <= float(dist_m) <= max_m:
            return float(dist_m)
        return None

    def _reset_altitude_pid(self, initial_alt: Optional[float] = None):
        self._alt_integral = 0.0
        self._prev_alt_m = initial_alt
        self._prev_alt_time = time.monotonic() if initial_alt is not None else None
        self._filtered_vz = 0.0
        self._stable_since = None
        self.last_thrust = self.HOVER_THRUST

    def _clamp_thrust(self, thrust: float) -> float:
        return max(self.MIN_THRUST, min(self.MAX_THRUST, thrust))

    def _compute_altitude_thrust(self, now: float) -> Tuple[float, Optional[float]]:
        """Computes continuous Z-axis PID thrust correction around HOVER_THRUST."""
        target_z = self.setpoint[2] if self.setpoint[2] > 0.05 else self.target_altitude_m
        cur_alt = self._get_current_altitude_m()

        if cur_alt is None:
            had_lock = self._prev_alt_m is not None
            self._prev_alt_m = None
            self._prev_alt_time = None
            self._stable_since = None
            base = (self.HOVER_THRUST + self._alt_integral) if had_lock else self.FALLBACK_CLIMB_THRUST
            self.last_thrust = self._clamp_thrust(base)
            return self.last_thrust, None

        if self._prev_alt_time is None:
            dt = 1.0 / max(1.0, self.target_hz)
        else:
            dt = max(1e-3, min(0.25, now - self._prev_alt_time))

        if self._prev_alt_m is not None:
            raw_vz = (cur_alt - self._prev_alt_m) / dt
            self._filtered_vz = 0.65 * self._filtered_vz + 0.35 * raw_vz
        else:
            self._filtered_vz = 0.0

        self._prev_alt_m = cur_alt
        self._prev_alt_time = now

        error = target_z - cur_alt

        # Proportional + Derivative on measurement velocity (avoids setpoint kick & damps overshoot)
        p_term = self.ALT_KP * error
        d_term = -self.ALT_KD * self._filtered_vz

        # Integral with anti-windup clamping
        candidate_i = max(-self.ALT_I_MAX, min(self.ALT_I_MAX, self._alt_integral + self.ALT_KI * error * dt))
        raw_thrust = self.HOVER_THRUST + p_term + candidate_i + d_term
        thrust = self._clamp_thrust(raw_thrust)

        if raw_thrust == thrust or (raw_thrust > thrust and error < 0.0) or (raw_thrust < thrust and error > 0.0):
            self._alt_integral = candidate_i

        self.last_thrust = thrust
        return thrust, cur_alt

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
                self._reset_altitude_pid()
                time.sleep(interval)
                continue

            now = time.monotonic()

            if self.phase == MissionPhase.ARMING:
                self._execute_arming()

            elif self.phase == MissionPhase.CLIMBING:
                thrust, cur_alt = self._compute_altitude_thrust(now)
                self.mavlink.send_attitude_target(0.0, 0.0, 0.0, thrust)

                target_z = self.setpoint[2] if self.setpoint[2] > 0.05 else self.target_altitude_m
                if cur_alt is not None:
                    alt_err = abs(target_z - cur_alt)
                    if alt_err <= self.ALT_TOLERANCE_M and abs(self._filtered_vz) <= self.VZ_STABLE_THRESH:
                        if self._stable_since is None:
                            self._stable_since = now
                        elif (now - self._stable_since) >= self.STABLE_HOLD_SEC:
                            next_phase = MissionPhase.ALIGNING if self.mission_mode else MissionPhase.HOVER
                            print(
                                f"[Control] Reference stabilized -> {next_phase.value} "
                                f"(alt={cur_alt:.2f}m, ref={target_z:.2f}m, thrust={thrust:.3f})"
                            )
                            self.phase = next_phase
                    else:
                        self._stable_since = None

            elif self.phase in {MissionPhase.HOVER, MissionPhase.ALIGNING}:
                thrust, _ = self._compute_altitude_thrust(now)
                self.mavlink.send_attitude_target(0.0, 0.0, 0.0, thrust)

            elif self.phase == MissionPhase.LANDING:
                self.command_land()

            time.sleep(interval)

    def _execute_arming(self):
        self.mavlink.set_mode("GUIDED_NOGPS")
        time.sleep(0.35)

        if not self.mavlink.state.is_armed():
            self.mavlink.arm()

        # Poll up to 2.5s for Pixhawk HEARTBEAT confirmation (runs at ~1Hz)
        deadline = time.time() + 2.5
        while time.time() < deadline and self.running and self.active_guidance:
            if self.mavlink.state.is_armed():
                break
            time.sleep(0.05)

        if not self.mavlink.state.is_armed():
            print("[Control] Arming confirmation timed out -> IDLE")
            self.mission_mode = False
            self.active_guidance = False
            self.phase = MissionPhase.IDLE
            return

        init_alt = self._get_current_altitude_m()
        self._reset_altitude_pid(initial_alt=init_alt)
        thrust, _ = self._compute_altitude_thrust(time.monotonic())
        print(f"[Control] Armed confirmed -> CLIMBING with Z-PID (ref={self.target_altitude_m:.2f}m, thrust={thrust:.3f})")
        self.phase = MissionPhase.CLIMBING


MasterControl = ControlManager