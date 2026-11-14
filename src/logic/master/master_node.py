# File: src/logic/master/master_node.py

import math
import random
import sys
import threading
import time
from typing import Optional, Tuple

from src.interfaces.MAVLink.master_mavlink import MAVLinkManager
from src.inputs.camera_driver.camera_state import CameraState
from src.inputs.distance_sensor.sensor_state import DistanceSensorState
from src.logic.control.control_state import ControlState
from src.logic.vision.vision_state import VisionState
from src.logic.master.master_state import MasterState, SystemState
from src.logic.master.command_guard import CommandGuard

# Real camera + vision managers (same facade standard as MAVLinkManager). If they
# cannot be imported (e.g. OpenCV missing), the camera/vision nodes fall back to
# flags only, as before (use 'sim on' for data).
try:
    from src.inputs.camera_driver.master_camera import CameraManager
    from src.logic.vision.master_vision import VisionManager
except Exception as _camera_import_error:  # noqa: BLE001
    CameraManager = VisionManager = None
    print(f"[MASTER Warning] Camera/vision managers unavailable: {_camera_import_error}")

try:
    from src.inputs.distance_sensor.master_sensor import DistanceSensorManager
except Exception as _distance_import_error:  # noqa: BLE001
    DistanceSensorManager = None
    print(f"[MASTER Warning] Distance sensor manager unavailable: {_distance_import_error}")


class MasterNode:
    """Central UAV Supervisor: manages subsystem lifecycles, safety guards, debug sim, and CLI control."""

    VALID_TARGETS = {"mavlink", "camera", "vision", "distance", "control", "all"}

    def __init__(self, mavlink_mode: str = "HARDWARE", enable_local_stdin: bool = True):
        self.mavlink_mode = mavlink_mode
        self.enable_local_stdin = enable_local_stdin

        # Core State & Safety Guard
        self.state = MasterState()
        self.guard = CommandGuard(self.state)

        # Subsystem instances (exposed so MasterWIFI can bind transmitters)
        self.mavlink_node = MAVLinkManager()
        # Camera + vision managers own their State objects; camera_state / vision_state
        # stay as the names MasterWIFI binds to.
        self.camera_manager = CameraManager() if CameraManager else None
        self.vision_manager = (VisionManager(self.camera_manager)
                               if VisionManager and self.camera_manager else None)
        self.camera_state = self.camera_manager.state if self.camera_manager else CameraState()
        self.vision_state = self.vision_manager.state if self.vision_manager else VisionState()
        self.distance_manager = DistanceSensorManager() if DistanceSensorManager else None
        self.distance_state = (self.distance_manager.state if self.distance_manager
                               else DistanceSensorState())
        self.control_state = ControlState()

        # Compatibility self-references for tx/*.py
        self.camera_state.state = self.camera_state
        self.vision_state.state = self.vision_state
        self.distance_state.state = self.distance_state
        self.control_state.state = self.control_state

        self._running = False
        self._sim_active = False
        self._stdin_thread: Optional[threading.Thread] = None
        self._sim_thread: Optional[threading.Thread] = None

    def start(self, auto_start_all: bool = True):
        """Boot MasterNode, optionally start all subsystems, and launch CLI listener."""
        self._running = True
        self.state.set_system_state(SystemState.BOOTING)
        self.state.append_cli_log("MasterNode initializing UAV subsystem orchestrator...", "SYSTEM")

        if auto_start_all:
            self.start_all_nodes()

        self.state.set_system_state(SystemState.IDLE)
        self._refresh_readiness_state()
        self.state.append_cli_log("MasterNode READY. Type 'help' for available commands.", "SYSTEM")

        if self.enable_local_stdin:
            self._stdin_thread = threading.Thread(target=self._local_cli_loop, daemon=True)
            self._stdin_thread.start()

    def stop(self):
        """Gracefully shut down all subsystems and MasterNode."""
        self._running = False
        self._sim_active = False
        self.state.append_cli_log("Shutting down all UAV subsystems...", "SYSTEM")
        self.stop_all_nodes(force=True)
        self.state.append_cli_log("MasterNode offline.", "SYSTEM")

    # =========================================================================
    # MAVLINK HARDWARE / SITL COMMAND BRIDGE
    # =========================================================================

    def _send_mavlink_command(self, action: str) -> Tuple[bool, str]:
        """Dispatches arm/disarm/mode/land/abort commands through MAVLinkManager."""
        mav = self.mavlink_node
        if mav is None:
            return False, "MAVLinkManager instance is None."

        # Check if MAVLink link is active (allow if SIM mode is on)
        is_connected = getattr(mav, "connected", True)
        if not is_connected and not self._sim_active:
            return False, "MAVLink link is OFFLINE (vehicle not connected)."

        tx = getattr(mav, "tx", None)
        targets = [obj for obj in (mav, tx) if obj is not None]

        # 1. Try high-level methods on MAVLinkManager or MAVLink TX
        method_candidates = {
            "arm": [("arm", ()), ("send_arm", ()), ("arm_drone", ()), ("arm_disarm", (True,)), ("set_armed", (True,))],
            "disarm": [("disarm", ()), ("send_disarm", ()), ("disarm_drone", ()), ("arm_disarm", (False,)), ("set_armed", (False,))],
            "mission_start": [("start_mission", ()), ("set_guided", ()), ("set_mode", ("OFFBOARD",)), ("set_mode", ("GUIDED",))],
            "land": [("land", ()), ("send_land", ()), ("land_drone", ()), ("set_mode", ("AUTO.LAND",)), ("set_mode", ("LAND",))],
            "abort": [("abort", ()), ("emergency_stop", ()), ("hold", ()), ("set_mode", ("AUTO.LOITER",)), ("set_mode", ("LOITER",))]
        }

        for target in targets:
            for method_name, args in method_candidates.get(action, []):
                fn = getattr(target, method_name, None)
                if callable(fn):
                    try:
                        res = fn(*args)
                        if res is False:
                            return False, f"MAVLink rejected '{action}' via {method_name}()."
                        return True, f"MAVLink '{action.upper()}' sent via {method_name}()."
                    except Exception as e:
                        return False, f"MAVLink error in {method_name}(): {e}"

        # 2. Fallback: Raw pymavlink COMMAND_LONG if connection object is exposed
        conn = getattr(mav, "master", None) or getattr(mav, "conn", None) or getattr(mav, "mav_conn", None)
        if conn is not None and hasattr(conn, "mav"):
            try:
                t_sys = getattr(conn, "target_system", 1)
                t_comp = getattr(conn, "target_component", 1)
                if action == "arm":
                    conn.mav.command_long_send(t_sys, t_comp, 400, 0, 1, 0, 0, 0, 0, 0, 0)
                    return True, "MAVLink MAV_CMD_COMPONENT_ARM_DISARM (ARM=1) sent."
                elif action == "disarm":
                    conn.mav.command_long_send(t_sys, t_comp, 400, 0, 0, 0, 0, 0, 0, 0, 0)
                    return True, "MAVLink MAV_CMD_COMPONENT_ARM_DISARM (ARM=0) sent."
                elif action == "land":
                    conn.mav.command_long_send(t_sys, t_comp, 21, 0, 0, 0, 0, 0, 0, 0, 0)
                    return True, "MAVLink MAV_CMD_NAV_LAND sent."
                elif action == "abort":
                    conn.mav.command_long_send(t_sys, t_comp, 176, 0, 1, 4, 0, 0, 0, 0, 0)
                    return True, "MAVLink HOLD/LOITER abort command sent."
                elif action == "mission_start":
                    return True, "MAVLink GUIDED/OFFBOARD mission active."
            except Exception as e:
                return False, f"Raw MAVLink send failed: {e}"

        if self._sim_active:
            return True, f"SIM mode active: '{action.upper()}' simulated."

        return False, f"No MAVLink '{action}' handler found in MAVLinkManager."

    # =========================================================================
    # SUBSYSTEM LIFECYCLE MANAGEMENT (START / STOP)
    # =========================================================================

    def start_node(self, name: str) -> bool:
        name = name.lower().strip()
        if name == "all":
            self.start_all_nodes()
            return True

        if name not in self.VALID_TARGETS:
            self.state.append_cli_log(
                f"Unknown node '{name}'. Valid: mavlink, camera, vision, distance, control, all",
                "ERROR"
            )
            return False

        subs = self.state.get_subsystems_snapshot()
        if subs[name]["running"]:
            self.state.append_cli_log(f"Node '{name}' is already RUNNING.", "WARN")
            return True

        try:
            if name == "mavlink":
                self.mavlink_node.start(mode=self.mavlink_mode)
            elif name == "camera":
                setattr(self.camera_state, "connected", True)
                setattr(self.camera_state, "current_rate_hz", 30.0)
                if self.camera_manager:
                    self.camera_manager.start(timeout=0)   # opens the camera in the background
            elif name == "vision":
                setattr(self.vision_state, "connected", True)
                setattr(self.vision_state, "current_rate_hz", 30.0)
                if self.vision_manager:
                    self.vision_manager.start(timeout=0)
                    if not self.camera_manager.running:
                        self.state.append_cli_log("Vision needs the camera node: run 'start camera'.", "WARN")
            elif name == "distance":
                setattr(self.distance_state, "connected", True)
                setattr(self.distance_state, "current_rate_hz", 15.0)
                if self.distance_manager:
                    self.distance_manager.start(timeout=0)  # opens the sensor in the background
            elif name == "control":
                setattr(self.control_state, "connected", True)
                setattr(self.control_state, "current_rate_hz", 20.0)

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
            if name == "mavlink":
                self.mavlink_node.stop()
            elif name == "camera":
                setattr(self.camera_state, "connected", False)
                if self.camera_manager:
                    self.camera_manager.stop()
            elif name == "vision":
                setattr(self.vision_state, "connected", False)
                if self.vision_manager:
                    self.vision_manager.stop()
            elif name == "distance":
                setattr(self.distance_state, "connected", False)
                if self.distance_manager:
                    self.distance_manager.stop()
            elif name == "control":
                setattr(self.control_state, "connected", False)

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
        """Automatically transitions between IDLE and READY when subsystems change."""
        cur = self.state.get_system_state()
        if cur in {SystemState.MISSION_ACTIVE, SystemState.LANDING, SystemState.EMERGENCY_ABORT}:
            return
        subs = self.state.get_subsystems_snapshot()
        all_up = all(v["running"] for v in subs.values())
        self.state.set_system_state(SystemState.READY if all_up else SystemState.IDLE)

    # =========================================================================
    # INTERACTIVE CLI PARSER (SSH & REMOTE GCS)
    # =========================================================================

    def execute_cli_command(self, raw_cmd: str) -> str:
        """Executes a text command from local SSH, remote cmd_master console, or ActionsView buttons."""
        cmd_clean = raw_cmd.strip()
        if not cmd_clean:
            return ""

        self.state.append_cli_log(f"> {cmd_clean}", "CMD")
        parts = cmd_clean.lower().split()
        verb = parts[0]
        arg = parts[1] if len(parts) > 1 else ""

        if verb == "help":
            help_msg = (
                "Commands: status | nodes | ping | sim <on|off> | start <node|all> | "
                "stop <node|all> | arm | disarm | mission start | mission land | abort | reset"
            )
            self.state.append_cli_log(help_msg, "INFO")
            return help_msg

        elif verb == "ping":
            self.state.append_cli_log("PONG! MasterNode link is alive (Read/Write OK).", "OK")
            return "PONG"

        elif verb == "sim":
            if arg == "on":
                self._start_debug_sim()
                return "SIM_ON"
            elif arg == "off":
                self._stop_debug_sim()
                return "SIM_OFF"
            else:
                self.state.append_cli_log("Usage: sim <on|off>", "ERROR")
                return "INVALID_SIM_ARG"

        elif verb == "status":
            subs = self.state.get_subsystems_snapshot()
            sys_st = self.state.get_system_state().value
            arm_st = "ARMED" if self.state.is_armed() else "DISARMED"
            mav_conn = "ONLINE" if getattr(self.mavlink_node, "connected", False) else "OFFLINE"
            summary = " | ".join([f"{k.upper()}:{'ON' if v['running'] else 'OFF'}" for k, v in subs.items()])
            msg = f"STATE=[{sys_st}] | {arm_st} | MAVLink={mav_conn} | SIM={'ON' if self._sim_active else 'OFF'} | {summary}"
            self.state.append_cli_log(msg, "STATUS")
            return msg

        elif verb in ("nodes", "info"):
            subs = self.state.get_subsystems_snapshot()
            mav_ok = getattr(self.mavlink_node, "connected", subs["mavlink"]["running"])
            aruco_locked = getattr(self.vision_state, "aruco_detected", False)
            aruco_id = getattr(self.vision_state, "aruco_id", -1)
            cam_res = getattr(self.camera_state, "resolution", "640x480")
            cam_fps = getattr(self.camera_state, "fps_actual", 0.0)

            self.state.append_cli_log("--- SUBSYSTEM DIAGNOSTICS ---", "SYSTEM")
            self.state.append_cli_log(f"1. MAVLINK  [Port 5005]: {'ONLINE' if mav_ok else 'OFFLINE'} | Mode={self.mavlink_mode}", "STATUS")
            self.state.append_cli_log(f"2. DISTANCE [Port 5002]: {'RUNNING' if subs['distance']['running'] else 'STOPPED'}", "STATUS")
            self.state.append_cli_log(f"3. CAMERA   [Port 5003]: {'RUNNING' if subs['camera']['running'] else 'STOPPED'} | {cam_res} @ {cam_fps}fps", "STATUS")
            self.state.append_cli_log(f"4. VISION   [Port 5006]: {'RUNNING' if subs['vision']['running'] else 'STOPPED'} | ArUco={'ID ' + str(aruco_id) if aruco_locked else 'SEARCHING'}", "STATUS")
            self.state.append_cli_log(f"5. CONTROL  [Port 5004]: {'RUNNING' if subs['control']['running'] else 'STOPPED'} | FSM={self.state.get_system_state().value}", "STATUS")
            return "NODES_INFO_OK"

        elif verb == "start":
            if not arg:
                self.state.append_cli_log("Usage: start <mavlink|camera|vision|distance|control|all>", "ERROR")
                return "MISSING_ARG"
            self.start_node(arg)
            return "OK"

        elif verb == "stop":
            if not arg:
                self.state.append_cli_log("Usage: stop <mavlink|camera|vision|distance|control|all>", "ERROR")
                return "MISSING_ARG"
            self.stop_node(arg)
            return "OK"

        elif verb == "arm":
            allowed, reason = self.guard.validate("arm")
            if not allowed:
                self.state.append_cli_log(reason, "BLOCKED")
                return reason
            ok, mav_msg = self._send_mavlink_command("arm")
            if not ok:
                self.state.append_cli_log(f"ARM FAILED: {mav_msg}", "ERROR")
                return mav_msg
            self.state.set_armed(True)
            self.state.append_cli_log(f"Vehicle ARMED ({mav_msg})", "WARN")
            return "ARMED"

        elif verb == "disarm":
            allowed, reason = self.guard.validate("disarm")
            if not allowed:
                self.state.append_cli_log(reason, "BLOCKED")
                return reason
            ok, mav_msg = self._send_mavlink_command("disarm")
            if not ok:
                self.state.append_cli_log(f"DISARM FAILED: {mav_msg}", "ERROR")
                return mav_msg
            self.state.set_armed(False)
            self.state.append_cli_log(f"Vehicle DISARMED ({mav_msg})", "INFO")
            return "DISARMED"

        elif verb == "mission":
            if arg == "start":
                allowed, reason = self.guard.validate("mission_start")
                if not allowed:
                    self.state.append_cli_log(reason, "BLOCKED")
                    return reason
                ok, mav_msg = self._send_mavlink_command("mission_start")
                if not ok:
                    self.state.append_cli_log(f"MISSION START FAILED: {mav_msg}", "ERROR")
                    return mav_msg
                self.state.set_system_state(SystemState.MISSION_ACTIVE)
                self.state.append_cli_log(f"Autonomous Precision Landing Mission STARTED ({mav_msg})", "OK")
                return "MISSION_ACTIVE"

            elif arg == "land":
                allowed, reason = self.guard.validate("mission_land")
                if not allowed:
                    self.state.append_cli_log(reason, "BLOCKED")
                    return reason
                ok, mav_msg = self._send_mavlink_command("land")
                if not ok:
                    self.state.append_cli_log(f"LAND FAILED: {mav_msg}", "ERROR")
                    return mav_msg
                self.state.set_system_state(SystemState.LANDING)
                self.state.append_cli_log(f"Landing sequence initiated ({mav_msg})", "OK")
                return "LANDING"
            else:
                self.state.append_cli_log("Usage: mission <start|land>", "ERROR")
                return "INVALID_MISSION_ARG"

        elif verb == "abort":
            ok, mav_msg = self._send_mavlink_command("abort")
            self.state.set_system_state(SystemState.EMERGENCY_ABORT)
            self.state.append_cli_log(f"EMERGENCY ABORT TRIGGERED! ({mav_msg})", "ALERT")
            return "EMERGENCY_ABORT"

        elif verb == "reset":
            self.state.set_armed(False)
            self.state.set_system_state(SystemState.IDLE)
            self._refresh_readiness_state()
            self.state.append_cli_log("System state reset to IDLE/READY.", "OK")
            return "RESET_OK"

        else:
            msg = f"Unknown command '{verb}'. Type 'help' for command list."
            self.state.append_cli_log(msg, "ERROR")
            return msg

    # =========================================================================
    # ON-DEMAND NODE DEBUGGING SIMULATOR ('sim on' / 'sim off')
    # =========================================================================

    def _start_debug_sim(self):
        if self._sim_active:
            self.state.append_cli_log("Debug simulator is already RUNNING.", "WARN")
            return
        self._sim_active = True
        self._sim_thread = threading.Thread(target=self._debug_sim_loop, daemon=True)
        self._sim_thread.start()
        self.state.append_cli_log("Debug telemetry simulator STARTED (injecting synthetic node data).", "OK")

    def _stop_debug_sim(self):
        if not self._sim_active:
            self.state.append_cli_log("Debug simulator is already OFF.", "WARN")
            return
        self._sim_active = False
        self.state.append_cli_log("Debug telemetry simulator STOPPED.", "WARN")

    def _debug_sim_loop(self):
        """Injects synthetic test values into active State objects when 'sim on' is enabled."""
        t0 = time.time()
        while self._running and self._sim_active:
            t = time.time() - t0
            subs = self.state.get_subsystems_snapshot()

            if subs["vision"]["running"]:
                self.vision_state.update_aruco_pose(
                    detected=True,
                    target_id=42,
                    x=0.10 * math.sin(t * 1.5) + random.uniform(-0.005, 0.005),
                    y=-0.08 * math.cos(t * 1.5) + random.uniform(-0.005, 0.005),
                    z=1.25 + 0.05 * math.sin(t),
                    roll=1.5 * math.sin(t),
                    pitch=-1.2 * math.cos(t),
                    yaw=2.0 * math.sin(t * 0.5),
                    tracking_state="LOCKED"
                )

            if subs["distance"]["running"]:
                sim_dist = 1.40 + 0.20 * math.sin(t * 1.2) + random.uniform(-0.005, 0.005)
                if hasattr(self.distance_state, "update_distance"):
                    self.distance_state.update_distance(sim_dist)
                elif hasattr(self.distance_state, "update"):
                    self.distance_state.update(sim_dist)

            if subs["control"]["running"]:
                act_x = 0.05 * math.sin(t * 2.0) + random.uniform(-0.003, 0.003)
                act_y = 0.05 * math.cos(t * 2.0) + random.uniform(-0.003, 0.003)
                act_z = 1.20 + 0.03 * math.sin(t)
                if hasattr(self.control_state, "update_control_state"):
                    self.control_state.update_control_state(
                        ref_x=0.0, ref_y=0.0, ref_z=1.2,
                        act_x=act_x, act_y=act_y, act_z=act_z
                    )
                elif hasattr(self.control_state, "update"):
                    self.control_state.update(
                        ref_x=0.0, ref_y=0.0, ref_z=1.2,
                        act_x=act_x, act_y=act_y, act_z=act_z
                    )

            time.sleep(0.05)

    def _local_cli_loop(self):
        """Allows typing commands directly inside an SSH session on the RPi."""
        while self._running:
            try:
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.2)
                    continue
                self.execute_cli_command(line)
            except Exception:
                time.sleep(0.2)