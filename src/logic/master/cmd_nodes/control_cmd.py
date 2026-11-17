# File: src/logic/master/cmd_nodes/control_cmd.py

from typing import List
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI
from src.logic.master.master_state import SystemState


class ControlCLI(BaseNodeCLI):
    """CLI commands for basic flight behaviors and autonomous guidance via ControlManager."""

    def __init__(self, manager, state, guard):
        super().__init__("control", manager, state, guard)

    def _register_commands(self):
        self.register_cmd(
            "takeoff",
            self.cmd_takeoff,
            "Basic flight: arm, takeoff [alt_m], and hold HOVER without alignment",
            global_alias=False,
        )
        self.register_cmd(
            "hover",
            self.cmd_hover,
            "Hold current HOVER position (cancels alignment if active)",
            aliases=["hold"],
            global_alias=True,
        )
        self.register_cmd(
            "land",
            self.cmd_land,
            "Stop guidance and execute landing",
            global_alias=False,
        )

    def cmd_status(self, _args: List[str]) -> str:
        healthy = self.manager.is_healthy()
        phase = getattr(self.manager.phase, "value", str(self.manager.phase))
        guidance = self.manager.active_guidance
        mission = self.manager.mission_mode
        self.log(
            f"healthy={healthy} | phase={phase} | guidance={guidance} | mission_mode={mission}",
            "STATUS",
        )
        return "CONTROL_STATUS"

    def cmd_takeoff(self, args: List[str]) -> str:
        allowed, reason = self.guard.validate("control_takeoff")
        if not allowed:
            self.log(reason, "BLOCKED")
            return reason

        alt = float(args[0]) if args else 1.5
        if not self.manager.is_healthy():
            self.manager.start()

        started = self.manager.start_basic_flight(altitude_m=alt)
        if not started:
            self.log("Basic takeoff failed: MAVLink unavailable.", "ERROR")
            return "CONTROL_TAKEOFF_FAILED"

        setattr(self.state, "in_controlled_flight", True)
        self.log(f"Basic flight started -> ARM + TAKEOFF ({alt:.2f}m) -> HOVER (no mission).", "OK")
        return "CONTROL_TAKEOFF_OK"

    def cmd_hover(self, _args: List[str]) -> str:
        if not self.manager.hold_hover():
            self.log("HOVER blocked: Vehicle must be ARMED first.", "BLOCKED")
            return "HOVER_BLOCKED"

        setattr(self.state, "in_controlled_flight", True)
        if self.state.get_system_state() == SystemState.MISSION_ACTIVE:
            self.state.set_system_state(SystemState.READY)
        self.log("ControlNode holding HOVER (mission alignment disabled).", "OK")
        return "HOVER_OK"

    def cmd_land(self, _args: List[str]) -> str:
        allowed, reason = self.guard.validate("mission_land")
        if not allowed:
            self.log(reason, "BLOCKED")
            return reason

        self.manager.command_land()
        setattr(self.state, "in_controlled_flight", False)
        self.state.set_system_state(SystemState.LANDING)
        self.log("ControlNode stopped guidance and commanded LAND.", "OK")
        return "CONTROL_LAND_OK"