# File: src/logic/master/master_node.py

import sys
import threading
import time
from typing import Any, Dict, Optional

from src.inputs.distance_sensor.master_sensor import DistanceManager
from src.interfaces.MAVLink.master_mavlink import MAVLinkManager
from src.logic.control.master_control import ControlManager, MissionPhase
from src.logic.master.cmd_nodes.base_cmd import BaseNodeCLI
from src.logic.master.cmd_nodes.camera_cmd import CameraCLI
from src.logic.master.cmd_nodes.control_cmd import ControlCLI
from src.logic.master.cmd_nodes.distance_cmd import DistanceCLI
from src.logic.master.cmd_nodes.mavlink_cmd import MAVLinkCLI
from src.logic.master.cmd_nodes.vision_cmd import VisionCLI
from src.logic.master.command_guard import CommandGuard
from src.logic.master.debug_sim import DebugSimulator
from src.logic.master.master_state import MasterState, SystemState
from src.logic.vision.master_vision import VisionManager

try:
    from src.inputs.camera_driver.master_camera import CameraManager
except ImportError:
    from src.inputs.camera_driver.camera_state import CameraState

    class CameraManager:  # type: ignore[no-redef]
        """Adapter exposing CameraState with standard lifecycle methods for MasterNode & CameraCLI."""

        def __init__(self):
            self.state = CameraState()
            self.connected: bool = False
            self.current_rate_hz: float = 0.0

        def start(self) -> bool:
            self.connected = True
            self.current_rate_hz = 30.0
            setattr(self.state, "connected", True)
            setattr(self.state, "current_rate_hz", 30.0)
            return True

        def stop(self):
            self.connected = False
            self.current_rate_hz = 0.0
            setattr(self.state, "connected", False)
            setattr(self.state, "current_rate_hz", 0.0)

        def is_healthy(self) -> bool:
            return self.connected


# Real camera + ArUco driver (falls back cleanly if OpenCV/camera hardware is unavailable)
try:
    from src.inputs.camera_driver.camera_node import CameraNode
except Exception as _camera_import_error:  # noqa: BLE001
    CameraNode = None
    print(f"[MASTER Warning] Camera node unavailable: {_camera_import_error}")


class MasterNode:
    """Central UAV Supervisor with strict RC-Master authority.

    The RC transmitter holds exclusive physical authority over arming and flight mode
    selection. Autonomous guidance is permitted ONLY while the vehicle is RC-armed
    and the RC flight-mode switch is set to GUIDED_NOGPS.
    """

    VALID_TARGETS = {"mavlink", "camera", "vision", "distance", "control", "all"}
    REQUIRED_AUTONOMOUS_MODE = "GUIDED_NOGPS"
    GCS_DISCONNECT_TIMEOUT_SEC = 10.0

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

        # Bind LiDAR & Optical Flow states to MAVLink 30Hz EKF3 sensor feed
        if hasattr(self.mavlink_node, "bind_sensor_states"):
            self.mavlink_node.bind_sensor_states(
                distance_state=self.distance_node.state,
                vision_state=self.vision_node.state,
            )

        # Hardware camera + ArUco + Optical Flow capture worker bound to shared states
        self._camera_driver = (
            CameraNode(self.camera_node.state, self.vision_node.state)
            if CameraNode is not None
            else None
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

        # Ground Station Wi-Fi link watchdog state
        self._last_gcs_contact: float = 0.0
        self._gcs_failsafe_triggered: bool = False
        self.wifi_bridge: Optional[Any] = None

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
        """Initializes all UAV subsystems and starts the hardware sync and CLI threads."""
        self._running = True
        self.state.set_system_state(SystemState.BOOTING)
        self.state.append_cli_log("MasterNode initializing UAV subsystem orchestrator (RC-Master mode)...", "SYSTEM")

        if auto_start_all:
            self.start_all_nodes()

        self.state.set_system_state(SystemState.IDLE)
        self._refresh_readiness_state()
        self.state.append_cli_log(
            "MasterNode READY. Arm via RC and set switch to GUIDED_NOGPS before running 'mission start'.",
            "SYSTEM",
        )

        self._sync_thread = threading.Thread(target=self._hardware_sync_loop, daemon=True)
        self._sync_thread.start()

        if self.enable_local_stdin:
            self._stdin_thread = threading.Thread(target=self._local_cli_loop, daemon=True)
            self._stdin_thread.start()

    def stop(self):
        """Stops all active subsystems and shuts down MasterNode cleanly."""
        self._running = False
        self.simulator.active = False
        self._pending_mission_confirm = False
        self.state.append_cli_log("Shutting down all UAV subsystems...", "SYSTEM")
        self.stop_all_nodes(force=True)
        self.state.append_cli_log("MasterNode offline.", "SYSTEM")

    def bind_wifi_bridge(self, bridge: Any):
        """Links MasterWIFI so its active heartbeat keeps the GCS link watchdog alive."""
        self.wifi_bridge = bridge

    def notify_gcs_activity(self):
        """Refreshes the Ground Station Wi-Fi link watchdog timestamp."""
        self._last_gcs_contact = time.time()
        if self._gcs_failsafe_triggered:
            self._gcs_failsafe_triggered = False
            self.state.append_cli_log("Ground Station Wi-Fi link RESTORED.", "OK")

    ARDUPILOT_MODE_MAP = {
        0: "STABILIZE",
        2: "ALT_HOLD",
        4: "GUIDED",
        5: "LOITER",
        6: "RTL",
        9: "LAND",
        16: "POSHOLD",
        20: "GUIDED_NOGPS",
    }

    def _get_pixhawk_mode(self) -> str:
        """Extracts and normalizes the active flight mode reported by Pixhawk via MAVLink."""
        st = self.mavlink_node.state
        hb = st.get_packet("HEARTBEAT") or {}

        raw_mode = getattr(st, "mode", None)
        if raw_mode is None:
            raw_mode = getattr(st, "flight_mode", None)
        if raw_mode is None:
            raw_mode = hb.get("custom_mode", hb.get("mode", ""))

        if isinstance(raw_mode, int) or (isinstance(raw_mode, str) and raw_mode.strip().isdigit()):
            return self.ARDUPILOT_MODE_MAP.get(int(raw_mode), f"MODE_{raw_mode}")

        return str(raw_mode).strip().upper()

    def _is_rc_master_authorized(self) -> tuple[bool, str]:
        """Checks whether the RC pilot has armed the vehicle and selected GUIDED_NOGPS."""
        if self.simulator.active:
            return True, "SIMULATOR_ACTIVE"

        if not self.mavlink_node.is_healthy():
            return False, "MAVLink link is OFFLINE."

        if not self.mavlink_node.state.is_armed():
            return False, "Vehicle is DISARMED. Arm manually with the RC transmitter first."

        current_mode = self._get_pixhawk_mode()
        if current_mode != self.REQUIRED_AUTONOMOUS_MODE:
            return (
                False,
                f"Flight mode is [{current_mode or 'UNKNOWN'}]. Flip RC switch to {self.REQUIRED_AUTONOMOUS_MODE} first.",
            )

        return True, "OK"

    def _check_gcs_link_failsafe(self):
        """Releases guidance and commands LAND if GCS link is lost for >10s while in autonomous mission."""
        if self.wifi_bridge is not None:
            pc_bridge = getattr(self.wifi_bridge, "pc_bridge", None)
            if pc_bridge is not None and getattr(pc_bridge, "is_gcs_connected", False):
                self._last_gcs_contact = time.time()
                if self._gcs_failsafe_triggered:
                    self._gcs_failsafe_triggered = False
                    self.state.append_cli_log("Ground Station Wi-Fi link RESTORED.", "OK")
                return

        if self._last_gcs_contact <= 0.0 or self._gcs_failsafe_triggered:
            return

        elapsed = time.time() - self._last_gcs_contact
        if elapsed < self.GCS_DISCONNECT_TIMEOUT_SEC:
            return

        sys_state = self.state.get_system_state()
        current_mode = self._get_pixhawk_mode()
        is_autonomous_airborne = (
            self.state.is_armed()
            and current_mode == self.REQUIRED_AUTONOMOUS_MODE
            and (self.control_node.is_airborne() or sys_state == SystemState.MISSION_ACTIVE)
        )

        if is_autonomous_airborne and sys_state != SystemState.LANDING:
            self._gcs_failsafe_triggered = True
            self._pending_mission_confirm = False
            self.control_node.command_land()
            setattr(self.state, "in_controlled_flight", False)
            self.state.set_system_state(SystemState.LANDING)
            self.state.append_cli_log(
                f"FAILSAFE: Ground Station disconnected for {elapsed:.1f}s! Executing autonomous LAND.",
                "ALERT",
            )

    def _hardware_sync_loop(self):
        """Continuously mirrors Pixhawk state and immediately yields control if RC overrides mode or disarms."""
        while self._running:
            if not self.simulator.active:
                if self.mavlink_node.is_healthy():
                    real_armed = bool(self.mavlink_node.state.is_armed())
                    current_mode = self._get_pixhawk_mode()

                    if self.state.is_armed() != real_armed:
                        self.state.set_armed(real_armed)

                    sys_state = self.state.get_system_state()
                    rc_mode_override = (
                        sys_state == SystemState.MISSION_ACTIVE
                        and current_mode != self.REQUIRED_AUTONOMOUS_MODE
                    )

                    # Immediately release RPi guidance if RC disarms OR pilot switches out of GUIDED_NOGPS
                    if not real_armed or rc_mode_override:
                        self.control_node.disable_guidance()
                        setattr(self.state, "in_controlled_flight", False)

                        if sys_state in {SystemState.LANDING, SystemState.MISSION_ACTIVE}:
                            self.state.set_system_state(SystemState.IDLE)
                            self._refresh_readiness_state()
                            if rc_mode_override:
                                self.state.append_cli_log(
                                    f"RC MASTER OVERRIDE: Mode switched to [{current_mode or 'MANUAL'}]. "
                                    "Autonomous guidance released to pilot.",
                                    "WARN",
                                )
                            elif not real_armed:
                                self.state.append_cli_log(
                                    "Vehicle DISARMED. Autonomous guidance stopped.",
                                    "INFO",
                                )
                else:
                    if self.state.is_armed():
                        self.state.set_armed(False)
                    self.control_node.disable_guidance()
                    setattr(self.state, "in_controlled_flight", False)
                    if self.state.get_system_state() in {SystemState.LANDING, SystemState.MISSION_ACTIVE}:
                        self.state.set_system_state(SystemState.IDLE)
                        self._refresh_readiness_state()

            setattr(self.state, "in_controlled_flight", self.control_node.is_airborne())
            self._check_gcs_link_failsafe()
            time.sleep(0.1)

    # =========================================================================
    # GLOBAL SUBSYSTEM LIFECYCLE MANAGEMENT (START / STOP)
    # =========================================================================

    def start_node(self, name: str) -> bool:
        """Starts an individual subsystem node or all nodes."""
        name = name.lower().strip()
        if name == "all":
            self.start_all_nodes()
            return True

        if name not in self.VALID_TARGETS:
            self.state.append_cli_log(
                f"Unknown node '{name}'. Valid: mavlink, camera, vision, distance, control, all",
                "ERROR",
            )
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

            if name == "camera" and self._camera_driver is not None:
                self._camera_driver.start()
            elif name == "vision" and self._camera_driver is not None:
                self._camera_driver.enable_vision(True)
                if not getattr(self._camera_driver, "is_running", False):
                    self.state.append_cli_log("Vision needs the camera node: run 'start camera'.", "WARN")

            self.state.update_subsystem(name, running=True, health="HEALTHY")
            self.state.append_cli_log(f"Node [{name.upper()}] -> STARTED.", "OK")
            self._refresh_readiness_state()
            return True
        except Exception as e:
            self.state.append_cli_log(f"Failed to start [{name.upper()}]: {e}", "ERROR")
            return False

    def stop_node(self, name: str, force: bool = False) -> bool:
        """Stops an individual subsystem node or all nodes."""
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
            if name == "camera" and self._camera_driver is not None:
                self._camera_driver.stop()
            elif name == "vision" and self._camera_driver is not None:
                self._camera_driver.enable_vision(False)

            self.node_clis[name].manager.stop()
            self.state.update_subsystem(name, running=False, health="OFFLINE")
            self.state.append_cli_log(f"Node [{name.upper()}] -> STOPPED.", "WARN")
            self._refresh_readiness_state()
            return True
        except Exception as e:
            self.state.append_cli_log(f"Error stopping [{name.upper()}]: {e}", "ERROR")
            return False

    def start_all_nodes(self):
        """Starts all UAV subsystem nodes in dependency order."""
        for node in ("mavlink", "distance", "camera", "vision", "control"):
            self.start_node(node)

    def stop_all_nodes(self, force: bool = False):
        """Stops all UAV subsystem nodes in reverse dependency order."""
        if not force:
            allowed, reason = self.guard.validate("stop", "all")
            if not allowed:
                self.state.append_cli_log(reason, "BLOCKED")
                return
        for node in ("control", "vision", "camera", "distance", "mavlink"):
            self.stop_node(node, force=True)

    def _refresh_readiness_state(self):
        """Updates system readiness between IDLE and READY when not actively flying or aborted."""
        cur = self.state.get_system_state()
        if cur in {SystemState.MISSION_ACTIVE, SystemState.LANDING, SystemState.EMERGENCY_ABORT}:
            return
        subs = self.state.get_subsystems_snapshot()
        all_up = all(v["running"] for v in subs.values())
        self.state.set_system_state(SystemState.READY if all_up else SystemState.IDLE)

    # =========================================================================
    # CENTRAL CLI ROUTER
    # =========================================================================

    def execute_cli_command(self, raw_cmd: str, from_gcs: bool = True) -> str:
        """Parses and routes global supervisor commands and per-node CLI instructions."""
        cmd_clean = raw_cmd.strip()
        if not cmd_clean:
            return ""

        if from_gcs:
            self.notify_gcs_activity()

        parts = cmd_clean.split()
        verb = parts[0].lower()
        rest = parts[1:]

        # Silent Ground Station link heartbeat (keeps 10s failsafe alive without spamming logs)
        if verb == "gcs_hb":
            self.notify_gcs_activity()
            return "GCS_HB_ACK"

        self.state.append_cli_log(f"> {cmd_clean}", "CMD")

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
                    "WARN",
                )
                return "AWAITING_CONFIRM"

        # Enforce EMERGENCY_ABORT lockout globally before routing
        if self.state.get_system_state() == SystemState.EMERGENCY_ABORT:
            allowed, reason = self.guard.validate(verb, rest[0] if rest else "")
            if not allowed:
                self.state.append_cli_log(reason, "BLOCKED")
                return reason

        # Enforce RC-Master exclusivity: RPi cannot arm motors directly in hardware mode
        if verb == "arm" and not self.simulator.active:
            msg = "BLOCKED: RC transmitter holds master authority. Arm manually with RC sticks."
            self.state.append_cli_log(msg, "BLOCKED")
            return msg

        # 1. Prefixed node command
        if verb in self.node_clis:
            if verb == "mavlink" and rest and rest[0].lower() == "arm" and not self.simulator.active:
                msg = "BLOCKED: RC transmitter holds master authority. Arm manually with RC sticks."
                self.state.append_cli_log(msg, "BLOCKED")
                return msg
            return self.node_clis[verb].execute_cli_command(rest)

        # 2. Global Supervisor Commands
        if verb == "help":
            msg = (
                "Global: status | nodes | ping | sim <on|off> | start <node|all> | stop <node|all> | "
                "mission <start|stop|land> | abort | reset\n"
                "RC-Master Prerequisite: Arm via RC sticks & set RC mode switch to GUIDED_NOGPS before 'mission start'.\n"
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
            mode_st = self._get_pixhawk_mode() or "UNKNOWN"
            summary = " | ".join([f"{k.upper()}:{'ON' if v['running'] else 'OFF'}" for k, v in subs.items()])
            msg = (
                f"STATE=[{sys_st}] | {arm_st} | MODE=[{mode_st}] | "
                f"SIM={'ON' if self.simulator.active else 'OFF'} | {summary}"
            )
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
                allowed, reason = self.guard.validate("mission_start")
                if not allowed:
                    self.state.append_cli_log(reason, "BLOCKED")
                    return reason

                rc_ok, rc_reason = self._is_rc_master_authorized()
                if not rc_ok:
                    self.state.append_cli_log(f"MISSION BLOCKED: {rc_reason}", "BLOCKED")
                    return "RC_AUTH_REQUIRED"

                self._pending_mission_alt = float(rest[1]) if len(rest) > 1 else 1.5
                self._pending_mission_confirm = True
                prompt = (
                    f"[CONFIRM] RC-Armed & {self.REQUIRED_AUTONOMOUS_MODE} verified. "
                    f"Mission will ascend to {self._pending_mission_alt:.2f}m and execute ArUco ALIGNING & PRECISION LANDING "
                    "(flip RC switch anytime to override). Continue? (y/n)"
                )
                self.state.append_cli_log(prompt, "WARN")
                return "AWAITING_CONFIRM"

            elif arg == "stop":
                self._pending_mission_confirm = False
                self.control_node.hold_hover()
                self.state.set_system_state(SystemState.READY)
                self.state.append_cli_log("Mission STOPPED -> Alignment cancelled, ControlNode holding HOVER.", "WARN")
                return "MISSION_STOPPED"

            elif arg == "land":
                allowed, reason = self.guard.validate("mission_land")
                if not allowed and not self.simulator.active:
                    self.state.append_cli_log(reason, "BLOCKED")
                    return reason

                self._pending_mission_confirm = False
                self.control_node.command_land()
                setattr(self.state, "in_controlled_flight", False)
                self.state.set_system_state(SystemState.LANDING)
                self.state.append_cli_log("Mission LAND -> Guidance stopped, Pixhawk landing.", "OK")
                return "LAND_SENT"

            self.state.append_cli_log("Usage: mission <start|stop|land>", "ERROR")
            return "INVALID_MISSION_ARG"

        elif verb == "abort":
            self._pending_mission_confirm = False
            self.control_node.abort_mission()
            setattr(self.state, "in_controlled_flight", False)
            self.node_clis["mavlink"].execute_cli_command(["mode", "LAND"])
            self.state.set_system_state(SystemState.EMERGENCY_ABORT)
            self.state.append_cli_log("EMERGENCY ABORT! Switched to LAND & disabled guidance.", "ALERT")
            return "EMERGENCY_ABORT"

        elif verb == "reset":
            self._pending_mission_confirm = False
            self._gcs_failsafe_triggered = False
            self.control_node.disable_guidance()
            setattr(self.state, "in_controlled_flight", False)
            real_armed = bool(self.mavlink_node.state.is_armed()) if self.mavlink_node.is_healthy() else False
            self.state.set_armed(real_armed)
            self.state.set_system_state(SystemState.IDLE)
            self._refresh_readiness_state()
            self.state.append_cli_log("System state reset to IDLE/READY.", "OK")
            return "RESET_OK"

        # 3. Dynamic Standalone Routing
        if verb == "land":
            self._pending_mission_confirm = False
            self.control_node.disable_guidance()
            setattr(self.state, "in_controlled_flight", False)

        for node_cli in self.node_clis.values():
            if node_cli.owns_command(verb):
                return node_cli.execute_cli_command([verb] + rest)

        msg = f"Unknown command '{verb}'. Type 'help' or '<node> help'."
        self.state.append_cli_log(msg, "ERROR")
        return msg

    def _launch_confirmed_mission(self) -> str:
        """Starts autonomous guidance ONLY after re-verifying RC-arm and GUIDED_NOGPS mode."""
        rc_ok, rc_reason = self._is_rc_master_authorized()
        if not rc_ok:
            self.state.append_cli_log(f"Mission ABORTED: {rc_reason}", "BLOCKED")
            return "MISSION_RC_AUTH_FAILED"

        if not self.control_node.is_healthy():
            self.start_node("control")

        self.control_node.start_mission(altitude_m=self._pending_mission_alt)
        self.state.set_system_state(SystemState.MISSION_ACTIVE)
        self.state.append_cli_log(
            f"Mission CONFIRMED -> Guidance ACTIVE (target {self._pending_mission_alt:.2f}m -> ArUco ALIGNING).",
            "OK",
        )
        return "MISSION_ACTIVE"

    def _local_cli_loop(self):
        """Reads commands from local stdin when enabled."""
        while self._running:
            try:
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.2)
                    continue
                self.execute_cli_command(line, from_gcs=False)
            except Exception:
                time.sleep(0.2)