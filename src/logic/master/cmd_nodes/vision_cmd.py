# File: src/logic/master/cmd_nodes/vision_cmd.py

import threading
import time
from typing import List
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI


# <repo>/experiments/vision/<date>/ (next to experiments/flights/, same layout)
class VisionCLI(BaseNodeCLI):
    """All VisionManager CLI commands and aliases."""

    def __init__(self, manager, state, guard, camera_driver=None, guided_sender=None):
        self.camera_driver = camera_driver  # CameraNode: owns the known-marker registry
        self.guided_sender = guided_sender  # GuidedSender: law -> Pixhawk, only in GUIDED
        super().__init__("vision", manager, state, guard)
        # Asks in the terminal when the target is a marker that is not registered
        self._asked = set()        # unknown labels already asked about (once each)
        self._asked_at = 0.0
        threading.Thread(target=self._ask_loop, name="VisionAsk", daemon=True).start()

    def _register_commands(self):
        self.register_cmd("marker", self.cmd_marker, "Set target ArUco marker ID <id>", aliases=["set_target_marker", "aruco_id"])
        self.register_cmd("pose", self.cmd_pose, "Read 3D relative pose (x, y, z, yaw) of target ArUco", aliases=["get_target_pose"])
        self.register_cmd("arucowatch", self.cmd_arucowatch,
                          "Live ArUco pose, 2 lines/s [seconds] (hand tests)", aliases=["vwatch"])
        self.register_cmd("arucoscan", self.cmd_arucoscan,
                          "Also look for markers of ANY family/ID (debug) <on|off>", aliases=["vscan"])
        self.register_cmd("visionlog", self.cmd_visionlog,
                          "Record the pixel target + lidar + attitude to CSV, 20 Hz [seconds] | stop",
                          aliases=["vlog"])
        self.register_cmd("landshadow", self.cmd_landshadow,
                          "Landing mission: status | reset | start | arm <align|approach|touchdown|hold> (sends ONLY in GUIDED) | stop | disarm",
                          aliases=["lshadow"])
        self.register_cmd("followunknown", self.cmd_followunknown,
                          "Allow going to a marker NOT in known_markers.yaml <yes|no|ask>", aliases=["fu"])
        self.register_cmd("markers", self.cmd_markers,
                          "List registered markers (priority order) [reload]", aliases=["known"])
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

    def cmd_arucoscan(self, args: List[str]) -> str:
        if not args or args[0].lower() not in ("on", "off"):
            on = bool(getattr(self.manager.state, "scan_any", False))
            self.log(f"arucoscan is {'ON' if on else 'OFF'}. Usage: arucoscan <on|off>", "STATUS")
            return "ARUCOSCAN_STATUS"
        on = args[0].lower() == "on"
        self.manager.state.scan_any = on
        if on:
            self.log("arucoscan ON: AprilTags and other marker families are found in a background "
                     "thread (all families 2 Hz, known ones 10 Hz; ArUco 4x4 at camera rate).", "OK")
        else:
            self.log("arucoscan OFF.", "OK")
        return "ARUCOSCAN_" + ("ON" if on else "OFF")

    def cmd_visionlog(self, args: List[str]) -> str:
        """Ask the ground station to record the vision packets (pixel target, lidar, tilt-
        compensated offset, roll/pitch) to experiments/vision/<date>/ ON THE LAPTOP. Nothing is
        written on the Pi: it only raises a flag in the vision telemetry."""
        state = self.manager.state
        if args and args[0].lower() == "stop":
            self._log_until = 0.0
            return "VISIONLOG_STOP"
        seconds = max(1.0, min(600.0, float(args[0]) if args else 60.0))
        if getattr(self, "_log_thread", None) and self._log_thread.is_alive():
            self.log("visionlog already recording (visionlog stop).", "WARN")
            return "VISIONLOG_BUSY"
        name = time.strftime("visionlog_%H-%M-%S")
        if self.camera_driver is not None:
            self.camera_driver.landing.reset()      # every recording starts the law from SEARCH
        self._log_until = time.time() + seconds
        state.vision_log = {"rec": True, "name": name}
        self._log_thread = threading.Thread(target=self._log_loop, args=(name,), name="VisionLog", daemon=True)
        self._log_thread.start()
        # The folder date is the LAPTOP's local date (the Pi runs on UTC, so it can differ)
        self.log(f"visionlog {seconds:.0f} s: the ground station records experiments/vision/"
                 f"<laptop date>/{name}.csv (keep the panel open); "
                 "landing law reset to SEARCH", "OK")
        return "VISIONLOG_STARTED"

    def _log_loop(self, name):
        while time.time() < self._log_until:
            time.sleep(0.1)
        self.manager.state.vision_log = {"rec": False, "name": name}
        self.log(f"visionlog finished ({name}.csv on the ground station).", "OK")

    def cmd_landshadow(self, args: List[str]) -> str:
        drv = self.camera_driver
        if args and args[0].lower() == "reset":
            if drv is None:
                self.log("No camera driver on this machine.", "ERROR")
                return "NO_CAMERA"
            drv.landing.reset()
            self.log("Landing law reset to SEARCH (shadow mode, nothing is sent).", "OK")
            return "LANDSHADOW_RESET"
        gs = self.guided_sender
        if args and args[0].lower() == "stop":
            if gs is None:
                self.log("No GUIDED sender on this machine.", "ERROR")
                return "NO_SENDER"
            in_guided = gs._guided_now()[0]
            gs.mission_stop()
            self.log("MISSION STOPPED: holding in GUIDED, stages disabled. Sticks return when you "
                     "leave GUIDED with ch5." if in_guided else "MISSION STOPPED: nothing is sent.", "OK")
            return "LANDSHADOW_STOP"
        if args and args[0].lower() == "start":
            if gs is None:
                self.log("No GUIDED sender on this machine.", "ERROR")
                return "NO_SENDER"
            gs.mission_start()
            if drv is not None:
                drv.landing.reset()
            self.log("MISSION STARTED: holding. Choose ALIGN, APPROACH or TOUCHDOWN.", "OK")
            return "LANDSHADOW_START"
        if args and args[0].lower() in ("arm", "disarm"):
            if gs is None:
                self.log("No GUIDED sender on this machine.", "ERROR")
                return "NO_SENDER"
            if args[0].lower() == "disarm":
                gs.disarm("pilot")
                self.log("Sender disarmed: law back to SHADOW, nothing is sent.", "OK")
                return "LANDSHADOW_DISARMED"
            # Moving between flying stages inside GUIDED keeps the law's progress
            keep = gs._guided_now()[0] and gs.stage in ("align", "descend", "land")
            was_aligning = gs.stage == "align"
            ok, msg = gs.select(args[1] if len(args) > 1 else "")
            if ok and was_aligning and gs.stage == "descend":
                self.log("Already aligning: keeps aligning and descends to 35 cm.", "OK")
            if ok and drv is not None and (not keep or gs.stage == "stop" or drv.landing.phase == "HOLD"):
                drv.landing.reset()         # start from SEARCH
            self.log(("Sender " + msg) if ok else f"NOT armed: {msg}. Usage: landshadow start, then arm <align|approach|touchdown|hold>",
                     "OK" if ok else "ERROR")
            return "LANDSHADOW_ARMED" if ok else "LANDSHADOW_ARM_REFUSED"
        self.log(self._shadow_text(), "STATUS")
        if gs is not None:
            st = gs.status()
            self.log(f"  sender: {gs.mode_word()}  in_guided={st['in_guided']}  sent={st['sent_count']}  "
                     f"last fwd={st['v_fwd'] * 100:+.1f} right={st['v_right'] * 100:+.1f} "
                     f"down={st['v_down'] * 100:+.1f} cm/s  ({st['reason']})  floor={gs.p.floor_h} m", "STATUS")
        if drv is not None:
            p = drv.landing.p
            self.log(f"  kp={p.kp} kd={p.kd} v_max={p.v_max} m/s  align<{p.align_tol * 100:.0f} cm for {p.align_hold_s} s  "
                     f"descend {p.descend_speed} m/s (pause > {p.pause_err * 100:.0f} cm)  final<{p.final_h} m "
                     f"@{p.final_speed} m/s  touchdown<{p.touchdown_h} m", "STATUS")
        return "LANDSHADOW_STATUS"

    def _shadow_text(self):
        ls = self.manager.state.get_landing_shadow() if hasattr(self.manager.state, "get_landing_shadow") else {}
        err = ls.get("err_m")
        h = ls.get("h_m")
        return (f"{ls.get('mode', 'SHADOW')} {ls.get('phase', '?'):<9} v fwd={ls.get('v_fwd', 0) * 100:+5.1f} right={ls.get('v_right', 0) * 100:+5.1f} "
                f"down={ls.get('v_down', 0) * 100:+5.1f} cm/s  err={'--' if err is None else f'{err * 100:4.1f}'} cm  "
                f"h={'--' if h is None else f'{h:.2f}'} m  [{ls.get('target') or '-'}] {ls.get('note', '')}")

    def cmd_followunknown(self, args: List[str]) -> str:
        state = self.manager.state
        word = args[0].lower() if args else ""
        value = {"yes": True, "y": True, "si": True, "true": True, "on": True,
                 "no": False, "n": False, "false": False, "off": False, "ask": None}
        if word not in value:
            cur = getattr(state, "follow_unknown", None)
            self.log(f"followunknown = {'ask (not decided)' if cur is None else ('YES' if cur else 'NO')}. "
                     "Usage: followunknown <yes|no|ask>", "STATUS")
            return "FOLLOWUNKNOWN_STATUS"
        state.follow_unknown = value[word]
        self._asked.clear()
        if value[word] is True:
            self.log("followunknown YES: if no registered marker is visible, the target is the "
                     "unknown marker nearest the centre (yellow).", "WARN")
        elif value[word] is False:
            self.log("followunknown NO: only registered markers (green) are followed.", "OK")
        else:
            self.log("followunknown reset: you will be asked again.", "OK")
        return "FOLLOWUNKNOWN_" + word.upper()

    def cmd_markers(self, args: List[str]) -> str:
        drv = self.camera_driver
        if drv is None:
            self.log("No camera driver on this machine.", "ERROR")
            return "NO_CAMERA"
        if args and args[0].lower() == "reload":
            drv.reload_registry()
            self.log("known_markers.yaml reloaded.", "OK")
        reg = list(drv.registry)
        if not reg:
            self.log("No registered markers (src/inputs/camera_driver/known_markers.yaml).", "STATUS")
        for i, label in enumerate(reg, 1):
            self.log(f"  {i}. {label}", "STATUS")
        return "MARKERS_OK"

    def _ask_loop(self):
        """When the chosen target is an unregistered marker and nobody decided yet, ask once per
        marker and at most every 10 s (the target can flip between unknowns). Nothing follows
        it until the answer is `followunknown yes`."""
        while True:
            time.sleep(0.5)
            try:
                state = self.manager.state
                pt = state.get_pixel_target()
                if not pt.get("detected") or pt.get("known") or getattr(state, "follow_unknown", None) is not None:
                    continue
                label = pt.get("source")
                now = time.time()
                if label and label not in self._asked and now - self._asked_at >= 10.0:
                    self._asked.add(label)
                    self._asked_at = now
                    self.log(f"Unregistered marker {label} is the target (yellow), no registered one "
                             f"in view. Go to it?  ->  followunknown yes  /  followunknown no", "WARN")
            except Exception:  # noqa: BLE001
                pass

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
        """Pixel target: x = right in the image, y = DOWN in the image (cm = angle x lidar
        height), z = lidar. Marker centre on the fixed cross -> x = y = 0."""
        state = self.manager.state
        while time.time() < self._watch_until:
            tel = state.export_telemetry_data()
            pt, st = tel.get("pixel_target", {}), tel["status"]
            if pt.get("detected"):
                if pt.get("x_m") is not None:
                    metric = (f"x={pt['x_m'] * 100:+6.1f} y={pt['y_m'] * 100:+6.1f} cm "
                              f"|{pt['err_m'] * 100:5.1f}| z={pt['h_m'] * 100:5.1f} cm(lidar)")
                    if pt.get("fwd_m") is not None:
                        metric += (f" || TILT-COMP fwd={pt['fwd_m'] * 100:+6.1f} right={pt['right_m'] * 100:+6.1f} cm"
                                   f" roll={pt['roll_deg']:+5.1f} pitch={pt['pitch_deg']:+5.1f}")
                    else:
                        metric += " || no attitude"
                else:
                    metric = "no lidar: no cm"
                kind = {"known": "KNOWN", "yes": "UNKNOWN-go", "no": "UNKNOWN-blocked",
                        "ask": "UNKNOWN-ask"}.get(pt.get("approval"), "?")
                self.log(f"{pt.get('source', '?'):<20} {kind:<15} px dx={pt['du_px']:+6.1f} dy={pt['dv_px']:+6.1f}  "
                         f"{metric}", "STATUS")
            else:
                self.log(f"no marker  [{st['tracking_state']}]", "STATUS")
            self.log("    " + self._shadow_text(), "STATUS")
            time.sleep(0.5)
        self.log("arucowatch finished.", "OK")
