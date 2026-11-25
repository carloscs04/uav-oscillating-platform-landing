# File: src/logic/master/cmd_nodes/distance_cmd.py

from typing import List
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI


class DistanceCLI(BaseNodeCLI):
    """All DistanceManager CLI commands and aliases."""

    def __init__(self, manager, state, guard):
        super().__init__("distance", manager, state, guard)

    def _register_commands(self):
        self.register_cmd("dist", self.cmd_read, "Read current downward distance in meters", aliases=["read", "get_distance_m"])
        self.register_cmd("lidargate", self.cmd_lidargate,
                          "Lidar spike gate: status | on | off (off = raw lidar, as before)", aliases=["lgate"])
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

    def cmd_lidargate(self, args: List[str]) -> str:
        st = getattr(self.manager, "state", None)
        gate = getattr(st, "gate", None)
        if gate is None:
            self.log("No lidar gate on this machine.", "ERROR")
            return "LIDARGATE_NONE"
        if args and args[0].lower() in ("on", "off"):
            gate.p.enabled = args[0].lower() == "on"
            gate.reset()
            self.log(f"lidargate {'ON' if gate.p.enabled else 'OFF (raw lidar to Pixhawk, camera and law)'}", "OK")
            return "LIDARGATE_" + ("ON" if gate.p.enabled else "OFF")
        snap = st.gate_snapshot()
        p = gate.p
        self.log(f"lidargate {'ON' if p.enabled else 'OFF'} | status {snap['status']} | raw {snap['raw_m'] * 100:.1f} cm"
                 f" -> used {st.distance_m * 100:.1f} cm ({st.sensor_status}) | rejected {snap['rejected_total']}"
                 f" samples in {snap['events']} episodes", "STATUS")
        self.log(f"   jump {p.jump_m * 100:.0f} cm, camera agree {p.cam_agree_m * 100:.0f} cm, no-camera max "
                 f"{p.est_max_s:.1f} s, settle {p.settle_s:.1f} s, cam = {p.cam_k:.3f} x lidar {p.cam_c_m * 100:+.1f} cm",
                 "STATUS")
        return "LIDARGATE_STATUS"

