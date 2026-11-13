# File: src/logic/master/cmd_nodes/vision_cmd.py

from typing import List
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI


class VisionCLI(BaseNodeCLI):
    """All VisionManager CLI commands and aliases."""

    def __init__(self, manager, state, guard):
        super().__init__("vision", manager, state, guard)

    def _register_commands(self):
        self.register_cmd("marker", self.cmd_marker, "Set target ArUco marker ID <id>", aliases=["set_target_marker", "aruco_id"])
        self.register_cmd("pose", self.cmd_pose, "Read 3D relative pose (x, y, z, yaw) of target ArUco", aliases=["get_target_pose"])
        self.register_cmd("locked", self.cmd_locked, "Check whether target ArUco marker is locked", aliases=["is_target_locked"])

    def cmd_status(self, _args: List[str]) -> str:
        pose = self.manager.get_target_pose()
        lock_str = f"LOCKED (ID {pose['id']})" if pose["detected"] else "SEARCHING"
        self.log(f"healthy={self.manager.is_healthy()} | target_id={self.manager.target_marker_id} | state={lock_str}", "STATUS")
        return "VISION_STATUS"

    def cmd_marker(self, args: List[str]) -> str:
        if not args:
            self.log("Usage: marker <marker_id>", "ERROR")
            return "MISSING_ID"
        marker_id = int(args[0])
        self.manager.set_target_marker(marker_id)
        self.log(f"Target ArUco marker ID updated to {marker_id}.", "OK")
        return "MARKER_OK"

    def cmd_pose(self, _args: List[str]) -> str:
        p = self.manager.get_target_pose()
        self.log(
            f"detected={p['detected']} | id={p['id']} | x={p['x']:.3f}m, y={p['y']:.3f}m, z={p['z']:.3f}m, yaw={p['yaw']:.1f}deg",
            "STATUS"
        )
        return "POSE_OK"

    def cmd_locked(self, _args: List[str]) -> str:
        locked = self.manager.is_target_locked()
        self.log(f"is_target_locked -> {locked}", "STATUS")
        return str(locked)