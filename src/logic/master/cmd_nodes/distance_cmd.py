# File: src/logic/master/cmd_nodes/distance_cmd.py

from typing import List
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI


class DistanceCLI(BaseNodeCLI):
    """All DistanceManager CLI commands and aliases."""

    def __init__(self, manager, state, guard):
        super().__init__("distance", manager, state, guard)

    def _register_commands(self):
        self.register_cmd("dist", self.cmd_read, "Read current downward distance in meters", aliases=["read", "get_distance_m"])
        self.register_cmd("inject_dist", self.cmd_inject, "Inject manual distance reading <meters>", aliases=["inject_reading"])

    def cmd_status(self, _args: List[str]) -> str:
        dist = self.manager.get_distance_m()
        self.log(f"healthy={self.manager.is_healthy()} | distance={dist:.3f} m", "STATUS")
        return "DISTANCE_STATUS"

    def cmd_read(self, _args: List[str]) -> str:
        dist = self.manager.get_distance_m()
        self.log(f"Current distance: {dist:.3f} m", "OK")
        return f"{dist:.3f}"

    def cmd_inject(self, args: List[str]) -> str:
        if not args:
            self.log("Usage: inject_dist <meters>", "ERROR")
            return "MISSING_DIST"
        val = float(args[0])
        self.manager.inject_reading(val)
        self.log(f"Manual distance reading injected: {val:.3f} m", "OK")
        return "INJECT_OK"