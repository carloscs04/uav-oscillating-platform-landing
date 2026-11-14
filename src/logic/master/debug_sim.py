# File: src/logic/master/debug_sim.py

import math
import random
import threading
import time
from typing import Optional


class DebugSimulator:
    """Injects synthetic telemetry into active subsystem managers for bench testing ('sim on/off')."""

    def __init__(self, master_node):
        self.master = master_node
        self.active: bool = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        if self.active:
            self.master.state.append_cli_log("Debug simulator is already RUNNING.", "WARN")
            return
        self.active = True
        self._thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._thread.start()
        self.master.state.append_cli_log("Debug telemetry simulator STARTED (synthetic data active).", "OK")

    def stop(self):
        if not self.active:
            self.master.state.append_cli_log("Debug simulator is already OFF.", "WARN")
            return
        self.active = False
        self.master.state.append_cli_log("Debug telemetry simulator STOPPED.", "WARN")

    def _sim_loop(self):
        t0 = time.time()
        while self.master._running and self.active:
            t = time.time() - t0
            subs = self.master.state.get_subsystems_snapshot()

            if subs["vision"]["running"]:
                self.master.vision_state.update_aruco_pose(
                    detected=True,
                    target_id=42,
                    x=0.10 * math.sin(t * 1.5) + random.uniform(-0.004, 0.004),
                    y=-0.08 * math.cos(t * 1.5) + random.uniform(-0.004, 0.004),
                    z=1.25 + 0.05 * math.sin(t),
                    roll=1.5 * math.sin(t),
                    pitch=-1.2 * math.cos(t),
                    yaw=2.0 * math.sin(t * 0.5),
                    tracking_state="LOCKED"
                )

            if subs["distance"]["running"]:
                sim_dist = 1.40 + 0.20 * math.sin(t * 1.2) + random.uniform(-0.004, 0.004)
                self.master.distance_node.inject_reading(sim_dist)

            if subs["control"]["running"] and hasattr(self.master.control_state, "update_control_state"):
                self.master.control_state.update_control_state(
                    ref_x=0.0, ref_y=0.0, ref_z=1.2,
                    act_x=0.05 * math.sin(t * 2.0),
                    act_y=0.05 * math.cos(t * 2.0),
                    act_z=1.20 + 0.03 * math.sin(t)
                )

            time.sleep(0.05)