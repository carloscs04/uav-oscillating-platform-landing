# File: src/logic/master/command_guard.py

from typing import Tuple
from src.logic.master.master_state import MasterState, SystemState


class CommandGuard:
    """Safety interlocks and state-machine policy validator for MasterNode commands."""

    CRITICAL_FLIGHT_NODES = {"mavlink", "control"}

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

        # 1. Emergency Abort Lockout: Only 'reset', 'status', 'help', or 'stop all' allowed
        if sys_state == SystemState.EMERGENCY_ABORT:
            if action not in {"reset", "status", "help", "stop"}:
                return False, "FORBIDDEN: System is in EMERGENCY_ABORT. Run 'reset' before issuing commands."

        # 2. Prevent stopping critical nodes mid-flight or while armed
        if action == "stop":
            if sys_state in {SystemState.MISSION_ACTIVE, SystemState.LANDING}:
                if target in self.CRITICAL_FLIGHT_NODES or target == "all":
                    return False, f"FORBIDDEN: Cannot stop '{target}' while state is {sys_state.value}. Use 'abort' first."
            if armed and (target == "mavlink" or target == "all"):
                return False, "FORBIDDEN: Cannot stop MAVLink while vehicle is ARMED. Disarm first."

        # 3. Arming rules
        if action == "arm":
            if not subsystems["mavlink"]["running"]:
                return False, "FORBIDDEN: Cannot ARM. MAVLink node is OFFLINE."
            if sys_state == SystemState.EMERGENCY_ABORT:
                return False, "FORBIDDEN: Clear EMERGENCY_ABORT with 'reset' before arming."

        # 4. Disarming rules
        if action == "disarm":
            if sys_state == SystemState.MISSION_ACTIVE:
                return False, "FORBIDDEN: Cannot DISARM during active mission. Land or abort first."

        # 5. Mission execution rules
        if action == "mission_start":
            if sys_state == SystemState.MISSION_ACTIVE:
                return False, "FORBIDDEN: Mission is already ACTIVE."
            missing = [
                name for name in ("mavlink", "control", "vision", "distance")
                if not subsystems[name]["running"]
            ]
            if missing:
                return False, f"FORBIDDEN: Cannot start mission. Offline required nodes: {', '.join(missing)}."
            if not armed:
                return False, "FORBIDDEN: Vehicle must be ARMED before starting autonomous mission."

        # 6. Landing rules
        if action == "mission_land":
            if not subsystems["mavlink"]["running"]:
                return False, "FORBIDDEN: MAVLink node is OFFLINE. Cannot command landing."

        return True, "OK"