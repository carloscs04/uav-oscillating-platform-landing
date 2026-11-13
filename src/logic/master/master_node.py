# File: src/logic/master/master_node.py

import sys
import threading
import time
from typing import Dict, Optional

from src.interfaces.MAVLink.master_mavlink import MAVLinkManager
from src.inputs.camera_driver.master_camera import CameraManager
from src.inputs.distance_sensor.master_distance import DistanceManager
from src.logic.vision.master_vision import VisionManager
from src.logic.control.master_control import ControlManager

from src.logic.master.master_state import MasterState, SystemState
from src.logic.master.command_guard import CommandGuard
from src.logic.master.debug_sim import DebugSimulator
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI
from src.logic.master.cmd_nodes.mavlink_cmd import MAVLinkCLI
from src.logic.master.cmd_nodes.camera_cmd import CameraCLI
from src.logic.master.cmd_nodes.vision_cmd import VisionCLI
from src.logic.master.cmd_nodes.distance_cmd import DistanceCLI
from src.logic.master.cmd_nodes.control_cmd import ControlCLI


class MasterNode:
    """Central UAV Supervisor: handles global lifecycle/mission commands and delegates node commands to cmd_nodes/."""

    VALID_TARGETS = {"mavlink", "camera", "vision", "distance", "control", "all"}

    def __init__(self, mavlink_mode: str = "HARDWARE", enable_local_stdin: bool = True):
        self.mavlink_mode = mavlink_mode
        self.enable_local_stdin = enable_local_stdin

        self.state = MasterState()
        self.guard = CommandGuard(self.state)

        # 1. Subsystem Facade Managers
        self.mavlink_node = MAVLinkManager()
        self.camera_node = CameraManager()
        self.distance_node = DistanceManager()
        self.vision_node = VisionManager()
        self.control_node = ControlManager(
            mavlink_node=self.mavlink_node,
            vision_node=self.vision_node,
            distance_node=self.distance_node,
        )

        # 2. Isolated Telemetry Simulator
        self.simulator = DebugSimulator(self)

        # 3. Modular Node CLI Routers
        self.node_clis: Dict[str, BaseNodeCLI] = {
            "mavlink": MAVLinkCLI(self.mavlink_node, self.state, self.guard, sim_ref=self.simulator),
            "camera": CameraCLI(self.camera_node, self.state, self.guard),
            "vision": VisionCLI(self.vision_node, self.state, self.guard),
            "distance": DistanceCLI(self.distance_node, self.state, self.guard),
            "control": ControlCLI(self.control_node, self.state, self.guard),
        }

        self._running = False
        self._pending_mission_confirm = False
        self._pending_mission_alt = 1.5
        self._stdin_thread: Optional[threading.Thread] = None
        self._sync_thread: Optional[threading.Thread] = None

    @property
    def camera_state(self):
        return self.camera_node.state

    @property
    def distance_state(self):
        return self.distance_node.state

    @property
    def vision_state(self):
        return self.vision_node.state

    @property
    def control_state(self):
        return self.control_node.state

    def start(self, auto_start_all: bool = True):
        self._running = True
        self.state.set_system_state(SystemState.BOOTING)
        self.state.append_cli_log("MasterNode initializing UAV subsystem orchestrator...", "SYSTEM")

        if auto_start_all:
            self.start_all_nodes()

        self.state.set_system_state(SystemState.IDLE)
        self._refresh_readiness_state()
        self.state.append_cli_log("MasterNode READY. Type 'help' or '<node> help' (e.g. 'mavlink help').", "SYSTEM")

        self._sync_thread = threading.Thread(target=self._hardware_sync_loop, daemon=True)
        self._sync_thread.start()

        if self.enable_local_stdin:
            self._stdin_thread = threading.Thread(target=self._local_cli_loop, daemon=True)
            self._stdin_thread.start()

    def stop(self):
        self._running = False
        self.simulator.active = False
        self._pending_mission_confirm = False
        self.state.append_cli_log("Shutting down all UAV subsystems...", "SYSTEM")
        self.stop_all_nodes(force=True)
        self.state.append_cli_log("MasterNode offline.", "SYSTEM")

    def _hardware_sync_loop(self):
        """Continuously mirrors real Pixhawk HEARTBEAT armed status into MasterState."""
        while self._running:
            if not self.simulator.active:
                if self.mavlink_node.is_healthy():
                    real_armed = bool(self.mavlink_node.state.is_armed())
                    if self.state.is_armed() != real_armed:
                        self.state.set_armed(real_armed)
                elif self.state.is_armed():
                    self.state.set_armed(False)
            time.sleep(0.2)

    # =========================================================================
    # GLOBAL SUBSYSTEM LIFECYCLE MANAGEMENT (START / STOP)
    # =========================================================================

    def start_node(self, name: str) -> bool:
        name = name.lower().strip()
        if name == "all":
            self.start_all_nodes()
            return True

        if name not in self.VALID_TARGETS:
            self.state.append_cli_log(f"Unknown node '{name}'. Valid: mavlink, camera, vision, distance, control, all", "ERROR")
            return False

        subs = self.state.get_subsystems_snapshot()
        if subs[name]["running"]:
            self.state.append_cli_log(f"Node '{name}' is already RUNNING.", "WARN")
            return True

        try:
            if name == "mavlink":
                self.mavlink_node.start(mode=self.mavlink_mode)
            else:
                self.node_clis[name].manager.start()

            self.state.update_subsystem(name, running=True, health="HEALTHY")
            self.state.append_cli_log(f"Node [{name.upper()}] -> STARTED.", "OK")
            self._refresh_readiness_state()
            return True
        except Exception as e:
            self.state.append_cli_log(f"Failed to start [{name.upper()}]: {e}", "ERROR")
            return False

    def stop_node(self, name: str, force: bool = False) -> bool:
        name = name.lower().strip()
        if name == "all":
            self.stop_all_nodes(force=force)
            return True

        if name not in self.VALID_TARGETS:
            self.state.append_cli_log(f"Unknown node '{name}'.", "ERROR")
            return False

        if not force:
            allowed, reason = self.guard.validate("stop", name)
            if not allowed:
                self.state.append_cli_log(reason, "BLOCKED")
                return False

        try:
            self.node_clis[name].manager.stop()
            self.state.update_subsystem(name, running=False, health="OFFLINE")
            self.state.append_cli_log(f"Node [{name.upper()}] -> STOPPED.", "WARN")
            self._refresh_readiness_state()
            return True
        except Exception as e:
            self.state.append_cli_log(f"Error stopping [{name.upper()}]: {e}", "ERROR")
            return False

    def start_all_nodes(self):
        for node in ("mavlink", "distance", "camera", "vision", "control"):
            self.start_node(node)

    def stop_all_nodes(self, force: bool = False):
        if not force:
            allowed, reason = self.guard.validate("stop", "all")
            if not allowed:
                self.state.append_cli_log(reason, "BLOCKED")
                return
        for node in ("control", "vision", "camera", "distance", "mavlink"):
            self.stop_node(node, force=True)

    def _refresh_readiness_state(self):
        cur = self.state.get_system_state()
        if cur in {SystemState.MISSION_ACTIVE, SystemState.LANDING, SystemState.EMERGENCY_ABORT}:
            return
        subs = self.state.get_subsystems_snapshot()
        all_up = all(v["running"] for v in subs.values())
        self.state.set_system_state(SystemState.READY if all_up else SystemState.IDLE)

    # =========================================================================
    # CENTRAL CLI ROUTER
    # =========================================================================

    def execute_cli_command(self, raw_cmd: str) -> str:
        cmd_clean = raw_cmd.strip()
        if not cmd_clean:
            return ""

        self.state.append_cli_log(f"> {cmd_clean}", "CMD")
        parts = cmd_clean.split()
        verb = parts[0].lower()
        rest = parts[1:]

        # 0. Terminal confirmation gate (blocks other commands until 'y' or 'n' is answered)
        if self._pending_mission_confirm:
            if verb in ("y", "yes", "confirm"):
                self._pending_mission_confirm = False
                return self._launch_confirmed_mission()
            elif verb in ("n", "no", "cancel"):
                self._pending_mission_confirm = False
                self.state.append_cli_log("Mission launch CANCELLED. Normal console commands resumed.", "WARN")
                return "MISSION_CANCELLED"
            elif verb != "abort":
                self.state.append_cli_log(
                    "Are you sure you want to continue? Type 'y' to launch or 'n' to cancel:",
                    "WARN"
                )
                return "AWAITING_CONFIRM"

        # 1. Prefixed node command
        if verb in self.node_clis:
            return self.node_clis[verb].execute_cli_command(rest)

        # 2. Global Supervisor Commands
        if verb == "help":
            msg = (
                "Global: status | nodes | ping | sim <on|off> | start <node|all> | stop <node|all> | "
                "mission <start|stop|land> | abort | reset\n"
                "Per-Node: <mavlink|camera|vision|distance|control> <help|status|methods|command...>"
            )
            self.state.append_cli_log(msg, "INFO")
            return msg

        elif verb == "ping":
            self.state.append_cli_log("PONG! MasterNode link alive.", "OK")
            return "PONG"

        elif verb == "sim":
            arg = rest[0].lower() if rest else ""
            if arg == "on":
                self.simulator.start()
                return "SIM_ON"
            elif arg == "off":
                self.simulator.stop()
                return "SIM_OFF"
            self.state.append_cli_log("Usage: sim <on|off>", "ERROR")
            return "INVALID_SIM_ARG"

        elif verb == "status":
            subs = self.state.get_subsystems_snapshot()
            sys_st = self.state.get_system_state().value
            arm_st = "ARMED" if self.state.is_armed() else "DISARMED"
            summary = " | ".join([f"{k.upper()}:{'ON' if v['running'] else 'OFF'}" for k, v in subs.items()])
            msg = f"STATE=[{sys_st}] | {arm_st} | SIM={'ON' if self.simulator.active else 'OFF'} | {summary}"
            self.state.append_cli_log(msg, "STATUS")
            return msg

        elif verb in ("nodes", "info"):
            for cli in self.node_clis.values():
                cli.cmd_status([])
            return "NODES_INFO_OK"

        elif verb == "start":
            if not rest:
                self.state.append_cli_log("Usage: start <mavlink|camera|vision|distance|control|all>", "ERROR")
                return "MISSING_ARG"
            self.start_node(rest[0])
            return "OK"

        elif verb == "stop":
            if not rest:
                self.state.append_cli_log("Usage: stop <mavlink|camera|vision|distance|control|all>", "ERROR")
                return "MISSING_ARG"
            self.stop_node(rest[0])
            return "OK"

        elif verb == "mission":
            arg = rest[0].lower() if rest else ""
            if arg == "start":
                if not self.mavlink_node.is_healthy() and not self.simulator.active:
                    self.state.append_cli_log("MISSION BLOCKED: MAVLink link is OFFLINE.", "BLOCKED")
                    return "MAVLINK_OFFLINE"

                self._pending_mission_alt = float(rest[1]) if len(rest) > 1 else 1.5
                self._pending_mission_confirm = True
                self.state.append_cli_log(
                    f"[CONFIRM] Mission will switch to GUIDED, ARM vehicle, and TAKEOFF to {self._pending_mission_alt:.2f}m (HOVER). "
                    "Are you sure you want to continue? (y/n)",
                    "WARN"
                )
                return "AWAITING_CONFIRM"

            elif arg == "stop":
                self._pending_mission_confirm = False
                self.control_node.hold_hover()
                self.control_node.disable_guidance()
                self.state.set_system_state(SystemState.READY)
                self.state.append_cli_log("Mission STOPPED -> Control guidance disabled, holding HOVER.", "WARN")
                return "MISSION_STOPPED"

            elif arg == "land":
                self._pending_mission_confirm = False
                self.control_node.disable_guidance()
                return self.node_clis["mavlink"].execute_cli_command(["land"])

            self.state.append_cli_log("Usage: mission <start|stop|land>", "ERROR")
            return "INVALID_MISSION_ARG"

        elif verb == "abort":
            self._pending_mission_confirm = False
            self.control_node.abort_mission()
            self.node_clis["mavlink"].execute_cli_command(["mode", "LOITER"])
            self.state.set_system_state(SystemState.EMERGENCY_ABORT)
            self.state.append_cli_log("EMERGENCY ABORT! Switched to LOITER & disabled guidance.", "ALERT")
            return "EMERGENCY_ABORT"

        elif verb == "reset":
            self._pending_mission_confirm = False
            self.control_node.disable_guidance()
            real_armed = bool(self.mavlink_node.state.is_armed()) if self.mavlink_node.is_healthy() else False
            self.state.set_armed(real_armed)
            self.state.set_system_state(SystemState.IDLE)
            self._refresh_readiness_state()
            self.state.append_cli_log("System state reset to IDLE/READY.", "OK")
            return "RESET_OK"

        # 3. Dynamic Standalone Routing
        for node_cli in self.node_clis.values():
            if node_cli.owns_command(verb):
                return node_cli.execute_cli_command([verb] + rest)

        msg = f"Unknown command '{verb}'. Type 'help' or '<node> help'."
        self.state.append_cli_log(msg, "ERROR")
        return msg

    def _launch_confirmed_mission(self) -> str:
        """Starts ControlManager and verifies Pixhawk actually arms before entering MISSION_ACTIVE."""
        if not self.control_node.is_healthy():
            self.start_node("control")

        self.control_node.start_mission(altitude_m=self._pending_mission_alt)

        if not self.simulator.active:
            time.sleep(1.3)
            if not self.mavlink_node.state.is_armed():
                st = self.mavlink_node.state.get_packet("STATUSTEXT") or {}
                reason = st.get("text", "Pre-arm check failed")
                self.control_node.disable_guidance()
                self.state.set_armed(False)
                self.state.set_system_state(SystemState.READY)
                self.state.append_cli_log(f"Mission ABORTED: Pixhawk refused to arm ({reason}).", "ERROR")
                return "MISSION_ARM_FAILED"

        self.state.set_armed(True)
        self.state.set_system_state(SystemState.MISSION_ACTIVE)
        self.state.append_cli_log(
            f"Mission CONFIRMED -> ControlNode active (ARMED + TAKEOFF {self._pending_mission_alt:.2f}m -> HOVER).",
            "OK"
        )
        return "MISSION_ACTIVE"

    def _local_cli_loop(self):
        while self._running:
            try:
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.2)
                    continue
                self.execute_cli_command(line)
            except Exception:
                time.sleep(0.2)