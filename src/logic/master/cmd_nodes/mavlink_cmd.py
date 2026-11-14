# File: src/logic/master/cmd_nodes/mavlink_cmd.py

import time
from typing import List
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI
from src.logic.master.master_state import SystemState


class MAVLinkCLI(BaseNodeCLI):
    """All MAVLinkManager & VehicleState CLI commands and aliases."""

    def __init__(self, manager, state, guard, sim_ref=None):
        self.sim_ref = sim_ref
        super().__init__("mavlink", manager, state, guard)

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

    def _is_sim_active(self) -> bool:
        return bool(self.sim_ref and getattr(self.sim_ref, "active", False))

    def _log_pixhawk_feedback(self):
        time.sleep(0.15)
        st = self.manager.state.get_packet("STATUSTEXT")
        if isinstance(st, dict) and st.get("text"):
            self.log(f"Pixhawk: {st['text']}", "STATUS")

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

        self.manager.arm()
        self.state.set_armed(True)
        self.log("ARM command sent to Pixhawk.", "WARN")
        self._log_pixhawk_feedback()
        return "ARMED"

    def cmd_force_arm(self, _args: List[str]) -> str:
        if not self.manager.is_healthy() and not self._is_sim_active():
            self.log("FORCE_ARM blocked: MAVLink is OFFLINE.", "ERROR")
            return "MAVLINK_OFFLINE"

        self.manager.debug_force_arm()
        self.state.set_armed(True)
        self.log("FORCE ARM command sent to Pixhawk (pre-arm checks bypassed).", "WARN")
        self._log_pixhawk_feedback()
        return "FORCE_ARMED"

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