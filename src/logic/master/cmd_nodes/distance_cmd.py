# File: src/logic/master/cmd_nodes/distance_cmd.py

import threading
import time
from typing import List
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI


class DistanceCLI(BaseNodeCLI):
    """All DistanceManager CLI commands and aliases."""

    def __init__(self, manager, state, guard):
        super().__init__("distance", manager, state, guard)

    def _register_commands(self):
        self.register_cmd("dist", self.cmd_read, "Read current downward distance in meters", aliases=["read", "get_distance_m"])
        self.register_cmd("lidargate", self.cmd_lidargate,
                          "Lidar spike gate: status | on | off | watch [s] | test [s] (bench, disarmed: gate active + watch) | test stop", aliases=["lgate"])
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
        if args and args[0].lower() == "test":
            if len(args) > 1 and args[1].lower() == "stop":
                st.test_until = 0.0
                self._watch_until = 0.0
                self.log("lidargate test stopped (gate back to LOITER/GUIDED armed only).", "OK")
                return "LIDARGATE_TEST_STOP"
            try:
                secs = max(10.0, min(600.0, float(args[1]) if len(args) > 1 else 120.0))
            except ValueError:
                self.log("Usage: lidargate test [seconds] | test stop", "ERROR")
                return "LIDARGATE_USAGE"
            st.test_until = time.time() + secs
            gate.reset()
            self.log(f"lidargate TEST {secs:.0f} s: gate active while DISARMED (ignored once armed). "
                     "Hold the drone level over the board, ArUco in view, and pass a slotted card "
                     "under the lidar: expect 'CAMERA corrects'. Then cover the camera and repeat: "
                     "expect 'ESTIMATE' and after 2 s 'UNRELIABLE'.", "OK")
            self._start_watch(st, secs)
            return "LIDARGATE_TEST"
        if args and args[0].lower() == "watch":
            secs = max(1.0, min(600.0, float(args[1]) if len(args) > 1 else 30.0))
            self._start_watch(st, secs)
            return "LIDARGATE_WATCH"
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

    def _start_watch(self, st, secs):
        self._watch_until = time.time() + secs
        t = getattr(self, "_watch_thread", None)
        if t is None or not t.is_alive():
            self._watch_thread = threading.Thread(target=self._gate_watch, args=(st,), daemon=True)
            self._watch_thread.start()

    def _gate_watch(self, st):
        while time.time() < self._watch_until:
            d = st.gate_snapshot()
            cm = lambda v: "  --  " if v is None else f"{v * 100:5.1f}"
            self.log(f"lidargate {d['status']:<10} decides: {d['source']:<9} | lidar {cm(d.get('raw_m'))} cm"
                     f" | camera {cm(d.get('cam_m'))} cm | used {cm(d.get('used_m'))} cm", "STATUS")
            time.sleep(0.5)
        self.log("lidargate watch finished.", "OK")

