# File: src/logic/master/command_guard.py

from typing import Tuple
from src.logic.master.master_state import MasterState, SystemState


class CommandGuard:
    """Safety interlocks and state-machine policy validator for MasterNode commands."""

    CRITICAL_FLIGHT_NODES = {"mavlink", "control", "camera"}
    AIRBORNE_STATES = {SystemState.MISSION_ACTIVE, SystemState.LANDING}

    def __init__(self, state: MasterState):
        self.state = state

    def validate(self, action: str, target: str = "") -> Tuple[bool, str]:
        """
        Evaluates whether a command is permitted in the current system state.
        Returns (is_allowed, reason_message).
        """
        sys_state = self.state.get_system_state()
        subsystems = self.state.get_subsystems_snapshot()
        armed = self.state.is_armed()
        in_controlled_flight = bool(getattr(self.state, "in_controlled_flight", False))
        airborne = armed and ((sys_state == SystemState.MISSION_ACTIVE) or in_controlled_flight)

        # 1. Emergency Abort Lockout
        if sys_state == SystemState.EMERGENCY_ABORT:
            if action not in {"reset", "status", "help", "stop", "disarm", "force_disarm"}:
                return False, "FORBIDDEN: System is in EMERGENCY_ABORT. Run 'reset' before issuing commands."

        # 2. Prevent stopping critical nodes mid-flight or while armed
        if action == "stop":
            if airborne and (target in self.CRITICAL_FLIGHT_NODES or target == "all"):
                return False, f"FORBIDDEN: Cannot stop '{target}' while airborne. Land first."
            if armed and (target in self.CRITICAL_FLIGHT_NODES or target == "all"):
                return False, f"FORBIDDEN: Cannot stop '{target}' while vehicle is ARMED. Disarm first."

        # 3. Arming rules
        if action == "arm":
            if not subsystems["mavlink"]["running"]:
                return False, "FORBIDDEN: Cannot ARM. MAVLink node is OFFLINE."
            if armed:
                return False, "FORBIDDEN: Vehicle is already ARMED."
            if sys_state == SystemState.LANDING:
                return False, "FORBIDDEN: System is in LANDING state. Run 'reset' or disarm first."

        # 4. Disarming rules (blocked during active flight, allowed once LAND has been commanded)
        if action == "disarm":
            if not armed:
                return False, "FORBIDDEN: Vehicle is already DISARMED."
            if airborne:
                return False, "FORBIDDEN: Cannot DISARM while airborne. Command 'land' or 'abort' first."

        # 5. Basic ControlNode takeoff rules
        if action == "control_takeoff":
            if sys_state in self.AIRBORNE_STATES:
                return False, f"FORBIDDEN: Cannot start basic takeoff while state is {sys_state.value}."
            missing = [
                name for name in ("mavlink", "control")
                if not subsystems[name]["running"]
            ]
            if missing:
                return False, f"FORBIDDEN: Offline required nodes: {', '.join(missing)}."

        # 6. Full Autonomous Mission start rules
        if action == "mission_start":
            if sys_state in self.AIRBORNE_STATES:
                return False, f"FORBIDDEN: Cannot start mission while state is {sys_state.value}."
            missing = [
                name for name in ("mavlink", "control", "camera")
                if not subsystems[name]["running"]
            ]
            if missing:
                return False, f"FORBIDDEN: Cannot start mission. Offline required nodes: {', '.join(missing)}."

        # 7. Landing rules
        if action == "mission_land":
            if not subsystems["mavlink"]["running"]:
                return False, "FORBIDDEN: MAVLink node is OFFLINE. Cannot command landing."
            if not armed:
                return False, "FORBIDDEN: Vehicle is already DISARMED."

        return True, "OK"