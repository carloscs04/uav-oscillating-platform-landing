# File: src/logic/master/cmd_nodes/camera_cmd.py

from typing import List
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI


class CameraCLI(BaseNodeCLI):
    """All CameraManager CLI commands and aliases."""

    def __init__(self, manager, state, guard):
        super().__init__("camera", manager, state, guard)

    def _register_commands(self):
        self.register_cmd("res", self.cmd_res, "Set camera resolution <width> <height>", aliases=["set_resolution", "resolution"])
        self.register_cmd("fps", self.cmd_fps, "Set target FPS <fps>", aliases=["set_fps"])
        self.register_cmd("frame_info", self.cmd_frame_info, "Check whether an active frame is in memory", aliases=["get_latest_frame"])

    def cmd_status(self, _args: List[str]) -> str:
        res = getattr(self.manager.state, "resolution", "640x480")
        fps = getattr(self.manager.state, "fps_actual", 0.0)
        self.log(f"healthy={self.manager.is_healthy()} | res={res} | fps={fps}", "STATUS")
        return "CAMERA_STATUS"

    def cmd_res(self, args: List[str]) -> str:
        if len(args) < 2:
            self.log("Usage: res <width> <height>", "ERROR")
            return "MISSING_RES"
        w, h = int(args[0]), int(args[1])
        self.manager.set_resolution(w, h)
        self.log(f"Resolution updated to {w}x{h}.", "OK")
        return "RES_OK"

    def cmd_fps(self, args: List[str]) -> str:
        if not args:
            self.log("Usage: fps <target_fps>", "ERROR")
            return "MISSING_FPS"
        fps = float(args[0])
        self.manager.set_fps(fps)
        self.log(f"Target FPS updated to {fps}.", "OK")
        return "FPS_OK"

    def cmd_frame_info(self, _args: List[str]) -> str:
        frame = self.manager.get_latest_frame()
        shape = getattr(frame, "shape", None) if frame is not None else "None"
        self.log(f"Latest frame shape={shape}", "STATUS")
        return str(shape)