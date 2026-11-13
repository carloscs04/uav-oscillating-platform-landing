# File: src/logic/master/cmd_nodes/control_cmd.py

from typing import List
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI


class ControlCLI(BaseNodeCLI):
    """All ControlManager CLI commands and aliases."""

    def __init__(self, manager, state, guard):
        super().__init__("control", manager, state, guard)

    def _register_commands(self):
        self.register_cmd("hover", self.cmd_hover, "Hold steady hover at current position", aliases=["hold_hover"])
        self.register_cmd("phase", self.cmd_phase, "Show current mission phase in ControlManager")
        self.register_cmd("guidance_on", self.cmd_enable, "Enable closed-loop guidance control", aliases=["enable_guidance", "ctrl_on"])
        self.register_cmd("guidance_off", self.cmd_disable, "Disable closed-loop guidance control", aliases=["disable_guidance", "ctrl_off"])
        self.register_cmd("setpoint", self.cmd_setpoint, "Set target offset setpoint <ref_x> <ref_y> <ref_z>", aliases=["set_target_offset", "sp"])

    def cmd_status(self, _args: List[str]) -> str:
        sp = self.manager.setpoint
        phase = getattr(self.manager.phase, "value", str(self.manager.phase))
        self.log(
            f"healthy={self.manager.is_healthy()} | phase={phase} | guidance={'ON' if self.manager.active_guidance else 'OFF'} | setpoint={sp}",
            "STATUS"
        )
        return "CONTROL_STATUS"

    def cmd_phase(self, _args: List[str]) -> str:
        phase = getattr(self.manager.phase, "value", str(self.manager.phase))
        self.log(f"Current control phase: {phase}", "STATUS")
        return phase

    def cmd_hover(self, _args: List[str]) -> str:
        if not self.state.is_armed():
            self.log("HOVER blocked: Vehicle is DISARMED (arm/takeoff failed or not executed).", "BLOCKED")
            return "HOVER_BLOCKED_DISARMED"

        if not self.manager.hold_hover():
            self.log("HOVER blocked: ControlManager refused hover transition.", "ERROR")
            return "HOVER_FAILED"

        self.log("HOVER command dispatched to ControlManager.", "OK")
        return "HOVER_OK"

    def cmd_enable(self, _args: List[str]) -> str:
        if not self.state.is_armed():
            self.log("Guidance blocked: Vehicle is DISARMED.", "BLOCKED")
            return "GUIDANCE_BLOCKED"
        self.manager.enable_guidance()
        self.log("Closed-loop guidance ENABLED.", "OK")
        return "GUIDANCE_ON"

    def cmd_disable(self, _args: List[str]) -> str:
        self.manager.disable_guidance()
        self.log("Closed-loop guidance DISABLED.", "WARN")
        return "GUIDANCE_OFF"

    def cmd_setpoint(self, args: List[str]) -> str:
        if len(args) < 3:
            self.log("Usage: setpoint <ref_x> <ref_y> <ref_z>", "ERROR")
            return "MISSING_SETPOINT"
        rx, ry, rz = float(args[0]), float(args[1]), float(args[2])
        self.manager.set_target_offset(rx, ry, rz)
        self.log(f"Setpoint updated to ({rx}, {ry}, {rz}).", "OK")
        return "SETPOINT_OK"