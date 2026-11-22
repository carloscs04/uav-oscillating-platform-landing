# File: src/logic/master/cmd_nodes/vision_cmd.py

import threading
import time
from typing import List
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI


class VisionCLI(BaseNodeCLI):
    """All VisionManager CLI commands and aliases."""

    def __init__(self, manager, state, guard):
        super().__init__("vision", manager, state, guard)

    def _register_commands(self):
        self.register_cmd("marker", self.cmd_marker, "Set target ArUco marker ID <id>", aliases=["set_target_marker", "aruco_id"])
        self.register_cmd("pose", self.cmd_pose, "Read 3D relative pose (x, y, z, yaw) of target ArUco", aliases=["get_target_pose"])
        self.register_cmd("arucowatch", self.cmd_arucowatch,
                          "Live ArUco pose, 2 lines/s [seconds] (hand tests)", aliases=["vwatch"])
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

    def cmd_arucowatch(self, args: List[str]) -> str:
        seconds = max(1.0, min(300.0, float(args[0]) if args else 30.0))
        self._watch_until = time.time() + seconds
        if getattr(self, "_watch_thread", None) and self._watch_thread.is_alive():
            self.log(f"arucowatch extended to {seconds:.0f} s.", "OK")
            return "ARUCOWATCH_EXTENDED"
        self._watch_thread = threading.Thread(target=self._watch_loop, daemon=True)
        self._watch_thread.start()
        return "ARUCOWATCH_STARTED"

    def _watch_loop(self):
        """Camera frame (OpenCV): x = right in the image, y = DOWN in the image, z = out of
        the lens; all in cm. Marker straight below the lens -> x = y = 0."""
        state = self.manager.state
        while time.time() < self._watch_until:
            tel = state.export_telemetry_data()
            p, st, ov = tel["aruco_pose"], tel["status"], tel.get("aruco_overlay", {})
            seen = ",".join(str(m["id"]) for m in ov.get("markers", [])) or "-"
            if p["detected"]:
                self.log(f"{st['tracking_state']:<9} id={p['id']:<3} x={p['x'] * 100:+6.1f} "
                         f"y={p['y'] * 100:+6.1f} z={p['z'] * 100:6.1f} cm  "
                         f"off={((p['x'] ** 2 + p['y'] ** 2) ** 0.5) * 100:5.1f} cm  yaw={p['yaw']:+6.1f}  "
                         f"seen=[{seen}]", "STATUS")
            else:
                self.log(f"{st['tracking_state']:<9} no marker  seen=[{seen}]", "STATUS")
            time.sleep(0.5)
        self.log("arucowatch finished.", "OK")
