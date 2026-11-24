# File: src/logic/master/cmd_nodes/mavlink_cmd.py

import threading
import time
from typing import List, Optional
from pymavlink import mavutil
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI
from src.logic.master.master_state import SystemState


class MAVLinkCLI(BaseNodeCLI):
    """All MAVLinkManager & VehicleState CLI commands and aliases."""

    # COMMAND_ACKs always shown; any other command's ACK is shown only when it failed
    USER_COMMANDS = {
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        401,  # MAV_CMD_RUN_PREARM_CHECKS
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        42007,  # MAV_CMD_SET_EKF_SOURCE_SET
    }
    SEVERITY_TAG = {0: "ERROR", 1: "ERROR", 2: "ERROR", 3: "ERROR", 4: "WARN", 5: "STATUS", 6: "INFO", 7: "INFO"}
    SEVERITY_NAME = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERR", 4: "WARN", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}

    def __init__(self, manager, state, guard, sim_ref=None):
        self.sim_ref = sim_ref
        super().__init__("mavlink", manager, state, guard)
        self._recent_texts = {}
        self._fwd_seq = 0
        threading.Thread(target=self._pixhawk_message_forwarder, name="PixhawkMsgToCLI", daemon=True).start()

    # --- Pixhawk messages -> master panel CLI ---
    def _vehicle_state(self):
        return getattr(self.manager, "state", None) if self.manager else None

    def _pixhawk_message_forwarder(self):
        """Prints every Pixhawk STATUSTEXT (PreArm/Arm reasons, EKF, failsafes...) and the
        COMMAND_ACK of user commands in the master panel, as they arrive."""
        while True:
            try:
                vs = self._vehicle_state()
                if vs is not None and hasattr(vs, "get_events_since"):
                    for seq, _t, kind, data in vs.get_events_since(self._fwd_seq):
                        self._fwd_seq = seq
                        if kind == "STATUSTEXT":
                            self._show_statustext(data)
                        elif kind == "COMMAND_ACK":
                            self._show_ack(data)
            except Exception as e:
                print(f"[MAVLINK_CLI] message forwarder error: {e}")
                time.sleep(1.0)
            time.sleep(0.1)

    def _show_statustext(self, data: dict):
        text = str(data.get("text", "")).strip()
        if not text:
            return
        now = time.time()
        if now - self._recent_texts.get(text, 0.0) < 5.0:   # same line repeated in a burst
            return
        self._recent_texts[text] = now
        if len(self._recent_texts) > 200:
            self._recent_texts = {k: v for k, v in self._recent_texts.items() if now - v < 60.0}
        sev = int(data.get("severity", 6))
        self.log(f"Pixhawk [{self.SEVERITY_NAME.get(sev, sev)}]: {text}", self.SEVERITY_TAG.get(sev, "INFO"))

    @staticmethod
    def _enum_name(enum: str, value: int, prefix: str) -> str:
        entry = mavutil.mavlink.enums.get(enum, {}).get(value)
        return entry.name.replace(prefix, "") if entry else str(value)

    def _show_ack(self, data: dict):
        cmd = int(data.get("command", -1))
        result = int(data.get("result", -1))
        ok = result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        if cmd not in self.USER_COMMANDS and ok:
            return
        cmd_name = {401: "RUN_PREARM_CHECKS", 42007: "SET_EKF_SOURCE_SET"}.get(
            cmd, self._enum_name("MAV_CMD", cmd, "MAV_CMD_"))
        res_name = self._enum_name("MAV_RESULT", result, "MAV_RESULT_")
        self.log(f"Pixhawk ACK: {cmd_name} -> {res_name}", "OK" if ok else "ERROR")

    def _wait_ack(self, command: int, after_seq: int, timeout: float = 2.0) -> Optional[int]:
        """MAV_RESULT of the first COMMAND_ACK for command after after_seq (None = no answer)."""
        vs = self._vehicle_state()
        if vs is None or not hasattr(vs, "get_events_since"):
            return None
        deadline = time.time() + timeout
        while time.time() < deadline:
            for _seq, _t, kind, data in vs.get_events_since(after_seq):
                if kind == "COMMAND_ACK" and int(data.get("command", -1)) == command:
                    return int(data.get("result", -1))
            time.sleep(0.05)
        return None

    def _event_seq(self) -> int:
        vs = self._vehicle_state()
        return vs.last_event_seq() if vs is not None and hasattr(vs, "last_event_seq") else 0

    def _report_arm_result(self, after_seq: int, label: str) -> bool:
        """Waits for the arm ACK + HEARTBEAT and says clearly whether the Pixhawk armed."""
        result = self._wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, after_seq)
        armed = False
        for _ in range(10):
            armed = self.manager.state.is_armed()
            if armed or result not in (None, mavutil.mavlink.MAV_RESULT_ACCEPTED):
                break
            time.sleep(0.1)
        if armed:
            self.log(f"{label}: Pixhawk is ARMED.", "OK")
            return True
        if result is None:
            self.log(f"{label}: no answer from the Pixhawk in 2 s (link? USB?). Not armed.", "ERROR")
        else:
            time.sleep(0.3)  # let the 'PreArm:'/'Arm:' texts print first
            self.log(f"{label} REJECTED ({self._enum_name('MAV_RESULT', result, 'MAV_RESULT_')}). "
                     f"Reason = the 'Pixhawk: PreArm/Arm ...' lines above. Type 'prearm' to list all checks.",
                     "ERROR")
        return False

    def _register_commands(self):
        self.register_cmd("arm", self.cmd_arm, "Standard Pixhawk arming")
        self.register_cmd("force_arm", self.cmd_force_arm, "Force arm Pixhawk (bypasses pre-arm checks)", aliases=["farm", "debug_force_arm"])
        self.register_cmd("disarm", self.cmd_disarm, "Standard Pixhawk disarming")
        self.register_cmd("force_disarm", self.cmd_force_disarm, "Force disarm Pixhawk immediately", aliases=["fdisarm", "debug_force_disarm"])
        self.register_cmd("mode", self.cmd_mode, "Set flight mode <GUIDED|LOITER|LAND|STABILIZE>", aliases=["set_mode"])
        self.register_cmd("takeoff", self.cmd_takeoff, "Autonomous takeoff [alt_m]")
        self.register_cmd("land", self.cmd_land, "Land at current position")
        self.register_cmd("vel", self.cmd_vel, "Send velocity setpoint <vx> <vy> <vz> in m/s", aliases=["send_velocity", "velocity"])
        self.register_cmd("watch", self.cmd_watch, "Inspect watched MAVLink telemetry packets", aliases=["packets"])
        self.register_cmd("add_pkt", self.cmd_add_pkt, "Subscribe to MAVLink packet <PKT>", aliases=["watch_add"])
        self.register_cmd("rm_pkt", self.cmd_rm_pkt, "Unsubscribe from MAVLink packet <PKT>", aliases=["watch_remove"])
        self.register_cmd("pkt", self.cmd_get_pkt, "Read latest value of packet <PKT>", aliases=["get_packet"])
        self.register_cmd("prearm", self.cmd_prearm, "Why won't it arm? Runs the Pixhawk pre-arm checks and lists failing sensors", aliases=["why", "checks"])
        self.register_cmd("calibrate", self.cmd_calibrate, "Pixhawk calibration, DISARMED only: calibrate <gyro|level|baro|all> (all = gyro, level, baro in order)", aliases=["cal"])
        self.register_cmd("baroflight", self.cmd_baroflight, "Baro flight prep, DISARMED only: heightsrc baro, then calibrate baro")
        self.register_cmd("heightsrc", self.cmd_heightsrc, "Height source: heightsrc [lidar|baro] (normal flights: lidar; over the platform: baro). Disarmed only", aliases=["hsrc"])
        self.register_cmd("flowcheck", self.cmd_flowcheck, "No-flight check: flow/rangefinder params, data and EKF state")
        self.register_cmd("flowtest", self.cmd_flowtest, "Rotation test [seconds]: rock the DISARMED drone, checks flow axes/scale/delay")
        self.register_cmd("flowwatch", self.cmd_flowwatch, "Live flow + EKF position, 1 line/s [seconds] (walk test)")

    def _is_sim_active(self) -> bool:
        return bool(self.sim_ref and getattr(self.sim_ref, "active", False))

    def _log_pixhawk_feedback(self):
        """Pixhawk texts/ACKs are printed by _pixhawk_message_forwarder; just give them time."""
        time.sleep(0.3)

    def cmd_status(self, _args: List[str]) -> str:
        healthy = self.manager.is_healthy()
        armed = self.manager.state.is_armed()
        hb = self.manager.state.get_packet("HEARTBEAT") or {}
        mode = hb.get("custom_mode", "UNKNOWN")
        self.log(f"link={'ONLINE' if healthy else 'OFFLINE'} | pixhawk_armed={armed} | mode={mode}", "STATUS")
        return "MAVLINK_STATUS"

    def cmd_arm(self, _args: List[str]) -> str:
        allowed, reason = self.guard.validate("arm")
        if not allowed:
            self.log(reason, "BLOCKED")
            return reason
        if not self.manager.is_healthy() and not self._is_sim_active():
            self.log("ARM blocked: MAVLink is OFFLINE.", "ERROR")
            return "MAVLINK_OFFLINE"

        seq = self._event_seq()
        self.manager.arm()
        self.log("ARM command sent to Pixhawk...", "WARN")
        if self._is_sim_active():
            self.state.set_armed(True)
            return "ARMED"
        ok = self._report_arm_result(seq, "ARM")
        self.state.set_armed(ok)
        return "ARMED" if ok else "ARM_REJECTED"

    def cmd_force_arm(self, _args: List[str]) -> str:
        if not self.manager.is_healthy() and not self._is_sim_active():
            self.log("FORCE_ARM blocked: MAVLink is OFFLINE.", "ERROR")
            return "MAVLINK_OFFLINE"

        seq = self._event_seq()
        self.manager.debug_force_arm()
        self.log("FORCE ARM command sent to Pixhawk (pre-arm checks bypassed)...", "WARN")
        if self._is_sim_active():
            self.state.set_armed(True)
            return "FORCE_ARMED"
        ok = self._report_arm_result(seq, "FORCE ARM")
        self.state.set_armed(ok)
        return "FORCE_ARMED" if ok else "FORCE_ARM_REJECTED"

    def cmd_disarm(self, _args: List[str]) -> str:
        allowed, reason = self.guard.validate("disarm")
        if not allowed:
            self.log(reason, "BLOCKED")
            return reason

        self.manager.disarm()
        self.state.set_armed(False)
        self.log("DISARM command sent to Pixhawk.", "INFO")
        return "DISARMED"

    def cmd_force_disarm(self, _args: List[str]) -> str:
        self.manager.debug_force_disarm()
        self.state.set_armed(False)
        self.log("FORCE DISARM command sent to Pixhawk.", "WARN")
        return "FORCE_DISARMED"

    def cmd_mode(self, args: List[str]) -> str:
        if not args:
            self.log("Usage: mode <GUIDED|STABILIZE|LOITER|LAND|ALT_HOLD>", "ERROR")
            return "MISSING_MODE"
        mode_name = args[0].upper()
        self.manager.set_mode(mode_name)
        self.log(f"Flight mode '{mode_name}' sent to Pixhawk.", "OK")
        self._log_pixhawk_feedback()
        return f"MODE_{mode_name}"

    def cmd_takeoff(self, args: List[str]) -> str:
        alt = float(args[0]) if args else 1.5
        self.manager.takeoff(alt)
        self.log(f"TAKEOFF ({alt} m) command sent.", "OK")
        self._log_pixhawk_feedback()
        return "TAKEOFF_SENT"

    def cmd_land(self, _args: List[str]) -> str:
        if not self.state.is_armed() and not self._is_sim_active():
            self.log("LAND blocked: Vehicle is DISARMED.", "BLOCKED")
            return "LAND_BLOCKED_DISARMED"

        self.manager.land()
        self.state.set_system_state(SystemState.LANDING)
        self.log("LAND command sent to Pixhawk.", "OK")
        return "LAND_SENT"
        
    def cmd_vel(self, args: List[str]) -> str:
        if len(args) < 3:
            self.log("Usage: vel <vx> <vy> <vz>", "ERROR")
            return "MISSING_VEL_ARGS"
        vx, vy, vz = float(args[0]), float(args[1]), float(args[2])
        self.manager.send_velocity(vx, vy, vz)
        self.log(f"Velocity setpoint sent: vx={vx}, vy={vy}, vz={vz} m/s", "OK")
        return "VEL_SENT"

    def cmd_watch(self, _args: List[str]) -> str:
        watched = self.manager.state.get_watched_packets()
        unwatched = self.manager.state.get_available_unwatched_packets()
        self.log(f"Watched: {watched} | Available: {unwatched}", "STATUS")
        return "WATCH_OK"

    def cmd_add_pkt(self, args: List[str]) -> str:
        if not args:
            self.log("Usage: add_pkt <PACKET_NAME>", "ERROR")
            return "MISSING_PKT"
        pkt = args[0].upper()
        self.manager.state.add_watched_packet(pkt)
        self.log(f"Packet '{pkt}' added to watch list.", "OK")
        return "ADDED"

    def cmd_rm_pkt(self, args: List[str]) -> str:
        if not args:
            self.log("Usage: rm_pkt <PACKET_NAME>", "ERROR")
            return "MISSING_PKT"
        pkt = args[0].upper()
        self.manager.state.remove_watched_packet(pkt)
        self.log(f"Packet '{pkt}' removed from watch list.", "OK")
        return "REMOVED"

    def cmd_get_pkt(self, args: List[str]) -> str:
        if not args:
            self.log("Usage: pkt <PACKET_NAME>", "ERROR")
            return "MISSING_PKT"
        pkt = args[0].upper()
        data = self.manager.state.get_packet(pkt)
        self.log(f"{pkt} -> {data}", "STATUS")
        return str(data)

    def cmd_prearm(self, _args: List[str]) -> str:
        if not self.manager.is_healthy():
            self.log("prearm: MAVLink is OFFLINE (no Pixhawk heartbeat).", "ERROR")
            return "MAVLINK_OFFLINE"
        vs = self.manager.state
        if vs.is_armed():
            self.log("prearm: the Pixhawk is already ARMED.", "OK")
        sys_st = vs.get_packet("SYS_STATUS") or {}
        present = int(sys_st.get("onboard_control_sensors_present", 0))
        enabled = int(sys_st.get("onboard_control_sensors_enabled", 0))
        health = int(sys_st.get("onboard_control_sensors_health", 0))
        bad = enabled & present & ~health
        names = []
        for bit, entry in mavutil.mavlink.enums.get("MAV_SYS_STATUS_SENSOR", {}).items():
            if bit and (bad & bit) and (bit & (bit - 1)) == 0:
                names.append(entry.name.replace("MAV_SYS_STATUS_", ""))
        if sys_st:
            self.log("Unhealthy sensors: " + (", ".join(names) if names else "none"),
                     "ERROR" if names else "OK")
        else:
            self.log("No SYS_STATUS received yet: cannot list sensor health.", "WARN")
        seq = self._event_seq()
        self.manager.run_prearm_checks()
        self.log("Pre-arm checks requested; failures arrive as 'Pixhawk: PreArm: ...' lines.", "STATUS")
        result = self._wait_ack(401, seq, timeout=1.5)
        time.sleep(0.5)
        failures = [d for _s, _t, k, d in vs.get_events_since(seq)
                    if k == "STATUSTEXT" and str(d.get("text", "")).startswith(("PreArm", "Arm"))]
        if failures:
            self.log(f"{len(failures)} pre-arm failure(s) reported (see lines above).", "ERROR")
            return "PREARM_FAIL"
        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.log("No PreArm failures reported: the Pixhawk should arm.", "OK")
            return "PREARM_OK"
        self.log("No PreArm text received (ACK="
                 f"{'none' if result is None else self._enum_name('MAV_RESULT', result, 'MAV_RESULT_')}).", "WARN")
        return "PREARM_UNKNOWN"

    CAL_HINTS = {
        "gyro": "Keep the drone completely STILL on the ground.",
        "level": "Drone must sit LEVEL on a flat surface (in its flying attitude) and still.",
        "baro": "Drone on the take-off spot, still, props off (sets this height as 0 m).",
    }

    CAL_ALL_ORDER = ("gyro", "level", "baro")

    def cmd_calibrate(self, args: List[str]) -> str:
        kind = args[0].lower() if args else ""
        if kind == "all":
            for k in self.CAL_ALL_ORDER:     # each waits for its ACK; stop at the first failure
                res = self.cmd_calibrate([k])
                if res != "CAL_OK":
                    self.log(f"calibrate all stopped at {k.upper()} ({res}).", "ERROR")
                    return res
            self.log("calibrate all: GYRO, LEVEL and BARO done.", "OK")
            return "CAL_OK"
        if kind not in self.CAL_HINTS:
            self.log("Usage: calibrate <gyro|level|baro>", "ERROR")
            return "CAL_USAGE"
        if not self.manager.is_healthy():
            self.log("calibrate: MAVLink is OFFLINE (no Pixhawk heartbeat). Is the Pixhawk powered?", "ERROR")
            return "MAVLINK_OFFLINE"
        if self.manager.state.is_armed():
            self.log("calibrate refused: vehicle is ARMED.", "BLOCKED")
            return "CAL_REFUSED_ARMED"
        self.log(f"Calibrating {kind.upper()}... {self.CAL_HINTS[kind]}", "WARN")
        seq = self._event_seq()
        if not self.manager.preflight_calibration(kind):
            self.log("calibrate: could not send the command.", "ERROR")
            return "CAL_SEND_FAILED"
        # gyro/level take a few seconds; the ACK arrives when they finish
        result = self._wait_ack(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, seq, timeout=15.0)
        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            if kind == "baro" and hasattr(self.manager.state, "reset_baro_reference"):
                self.manager.state.reset_baro_reference()   # CSV baro altitude restarts at 0
            self.log(f"{kind.upper()} calibration done.", "OK")
            return "CAL_OK"
        if result is None:
            self.log(f"{kind.upper()} calibration: no answer in 15 s. Check the 'Pixhawk:' lines above.", "WARN")
            return "CAL_NO_ACK"
        self.log(f"{kind.upper()} calibration FAILED "
                 f"({self._enum_name('MAV_RESULT', result, 'MAV_RESULT_')}). Reason in the 'Pixhawk:' lines above.", "ERROR")
        return "CAL_FAILED"

    HEIGHT_SOURCES = {"lidar": 1, "baro": 2}

    def cmd_heightsrc(self, args: List[str]) -> str:
        vs = self.manager.state
        cur = vs.get_height_source_set() if hasattr(vs, "get_height_source_set") else 0
        cur_name = vs.HEIGHT_SOURCE_NAMES.get(cur, cur) if hasattr(vs, "HEIGHT_SOURCE_NAMES") else cur
        if not args:
            self.log(f"Active height source: {cur_name}. Use 'heightsrc lidar' (normal) or "
                     f"'heightsrc baro' (over the platform).", "STATUS")
            return f"HEIGHTSRC_{cur_name}"
        name = args[0].lower()
        if name not in self.HEIGHT_SOURCES:
            self.log("Usage: heightsrc [lidar|baro]", "ERROR")
            return "HEIGHTSRC_USAGE"
        if not self.manager.is_healthy():
            self.log("heightsrc: MAVLink is OFFLINE (no Pixhawk heartbeat).", "ERROR")
            return "MAVLINK_OFFLINE"
        if vs.is_armed():
            self.log("heightsrc refused: vehicle is ARMED. Change it on the ground, disarmed.", "BLOCKED")
            return "HEIGHTSRC_REFUSED_ARMED"
        want = self.HEIGHT_SOURCES[name]
        seq = self._event_seq()
        if not self.manager.set_ekf_source_set(want):
            self.log("heightsrc: could not send the command.", "ERROR")
            return "HEIGHTSRC_SEND_FAILED"
        result = self._wait_ack(42007, seq, timeout=2.0)
        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            vs.set_height_source_set(want)
            self.log(f"Height source is now {name.upper()} (EKF source set {want}).", "OK")
            if name == "baro":
                self.log("Run 'calibrate baro' now if you did not on this take-off spot. "
                         "A Pixhawk reboot or Pi restart goes back to LIDAR.", "WARN")
            return f"HEIGHTSRC_{name.upper()}"
        why = "no answer in 2 s" if result is None else self._enum_name("MAV_RESULT", result, "MAV_RESULT_")
        self.log(f"heightsrc {name} FAILED ({why}). Height source unchanged: {cur_name}.", "ERROR")
        return "HEIGHTSRC_FAILED"

    def cmd_baroflight(self, _args: List[str]) -> str:
        res = self.cmd_heightsrc(["baro"])
        if res != "HEIGHTSRC_BARO":
            self.log(f"baroflight stopped: height source not changed ({res}).", "ERROR")
            return res
        return self.cmd_calibrate(["baro"])

    def cmd_flowcheck(self, _args: List[str]) -> str:
        if not self.manager.is_healthy():
            self.log("flowcheck: MAVLink is OFFLINE.", "ERROR")
            return "MAVLINK_OFFLINE"
        lines = self.manager.flow_check()
        for tag, text in lines:
            self.log(text, tag)
        return f"FLOWCHECK_{lines[-1][0]}"

    def cmd_flowtest(self, args: List[str]) -> str:
        if not self.manager.is_healthy():
            self.log("flowtest: MAVLink is OFFLINE.", "ERROR")
            return "MAVLINK_OFFLINE"
        seconds = float(args[0]) if args else 15.0
        ok = self.manager.flow_test(max(5.0, min(60.0, seconds)), self.log)
        return "FLOWTEST_STARTED" if ok else "FLOWTEST_REFUSED"

    def cmd_flowwatch(self, args: List[str]) -> str:
        seconds = float(args[0]) if args else 30.0
        self.manager.flow_watch(max(1.0, min(300.0, seconds)), self.log)
        return "FLOWWATCH_STARTED"

