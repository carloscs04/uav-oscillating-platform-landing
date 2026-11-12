# File: src/interfaces/Bridge_WIFI/master_wifi.py

import os
import sys
import time
from typing import Dict, Any, Set, Optional

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rx.pc_bridge import PCBridgeListener
from rx.master_node import MasterCommandReceiver
from tx.control_node import ControlTransmitter
from tx.camera import CameraTransmitter
from tx.vision import VisionTransmitter
from tx.mavlink import MavlinkTransmitter
from tx.distance_sensor import DistanceSensorTransmitter
from tx.master_node import MasterTransmitter


class MasterWIFI:
    """Master Telecommunications Orchestrator for Raspberry Pi.

    Manages connection timeouts, global stream pause/resumes, per-packet filters,
    routes CLI/mission commands to MasterNode, and forwards live telemetry over UDP.
    """

    def __init__(
        self,
        handshake_port: int = 5007,
        master_rx_port: int = 5009,
        mavlink_node: Optional[Any] = None,
        control_node: Optional[Any] = None,
        camera_node: Optional[Any] = None,
        distance_node: Optional[Any] = None,
        master_node: Optional[Any] = None,
        vision_node: Optional[Any] = None
    ):
        self.handshake_port = handshake_port
        self.master_rx_port = master_rx_port

        # Subsystem Node References
        self.mavlink_node = mavlink_node
        self.control_node = control_node
        self.camera_node = camera_node
        self.distance_node = distance_node
        self.master_node = master_node
        self.vision_node = vision_node

        self.target_ip: Optional[str] = None
        self.tx_nodes: Dict[str, Any] = {}
        self.manually_disabled_nodes: Set[str] = set()

        self.pc_bridge = PCBridgeListener(
            port=self.handshake_port,
            timeout_sec=3.0,
            on_ip_discovered=self._on_gcs_connected,
            on_connection_lost=self._on_gcs_disconnected,
            on_command_received=self._on_command_received
        )

        # Dedicated CLI receiver for MasterNode
        self.master_cmd_rx = MasterCommandReceiver(
            port=self.master_rx_port,
            master_node=self.master_node
        )

    def set_mavlink_node(self, node: Any):
        self.mavlink_node = node
        if "mavlink_tx" in self.tx_nodes:
            self.tx_nodes["mavlink_tx"].mavlink_node = node

    def set_control_node(self, node: Any):
        self.control_node = node
        if "control_tx" in self.tx_nodes:
            self.tx_nodes["control_tx"].control_node = node

    def set_camera_node(self, node: Any):
        self.camera_node = node
        if "camera_tx" in self.tx_nodes:
            self.tx_nodes["camera_tx"].camera_node = node

    def set_distance_node(self, node: Any):
        self.distance_node = node
        if "distance_tx" in self.tx_nodes:
            self.tx_nodes["distance_tx"].distance_node = node

    def set_vision_node(self, node: Any):
        self.vision_node = node
        if "vision_tx" in self.tx_nodes:
            self.tx_nodes["vision_tx"].vision_node = node

    def set_master_node(self, node: Any):
        self.master_node = node
        self.master_cmd_rx.set_master_node(node)
        if "master_tx" in self.tx_nodes:
            self.tx_nodes["master_tx"].master_node = node

    def start(self):
        print("==================================================")
        print("   STARTING RPI MASTER TELECOM (Bridge-WIFI)      ")
        print("==================================================")

        self.pc_bridge.start()
        self.master_cmd_rx.start()
        print("[MASTER_TELECOM] Awaiting GCS handshake and commands...")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[MASTER_TELECOM] Shutting down services...")
            self.stop()

    def _on_gcs_connected(self, gcs_ip: str):
        print(f"[MASTER_TELECOM] GCS ONLINE -> IP: {gcs_ip}. Resuming active stream nodes...")
        self.target_ip = gcs_ip
        self._start_or_update_tx_nodes(gcs_ip)
        self.resume_all_active_nodes()

        if self.master_node and hasattr(self.master_node, "state"):
            self.master_node.state.append_cli_log(
                f"GCS Bridge linked ({gcs_ip}). Telemetry (5001-5008) & CLI RX (5009) active.",
                "SYSTEM"
            )

    def _on_gcs_disconnected(self):
        print("[MASTER_TELECOM] GCS OFFLINE -> Pausing ALL telemetry transmitter nodes...")
        self.pause_all_nodes()

    def pause_all_nodes(self):
        for name, node in self.tx_nodes.items():
            if hasattr(node, "pause_stream"):
                node.pause_stream()

    def resume_all_active_nodes(self):
        for name, node in self.tx_nodes.items():
            if name not in self.manually_disabled_nodes and hasattr(node, "resume_stream"):
                node.resume_stream()

    def _on_command_received(self, cmd: str, payload: Dict[str, Any], gcs_ip: str):
        """Routes incoming GCS telemetry stream controls and MasterNode CLI/mission commands."""
        payload = payload if isinstance(payload, dict) else {}
        nested_data = payload.get("data") if isinstance(payload.get("data"), dict) else {}

        # 1. Route Remote Terminal CLI & Mission Actions directly to MasterNode
        if self.master_node and hasattr(self.master_node, "execute_cli_command"):
            if cmd in ("CLI_CMD", "TERMINAL_CMD", "MASTER_CMD", "EXEC_CMD"):
                raw_text = (
                    payload.get("text")
                    or payload.get("command")
                    or payload.get("cli_command")
                    or nested_data.get("text")
                    or nested_data.get("command")
                    or ""
                )
                if raw_text:
                    self.master_node.execute_cli_command(str(raw_text))
                return

            action_map = {
                "ARM_DRONE": "arm",
                "DISARM_DRONE": "disarm",
                "START_MISSION": "mission start",
                "LAND_DRONE": "mission land",
                "ABORT": "abort",
                "EMERGENCY_ABORT": "abort",
                "RESET_SYSTEM": "reset"
            }
            if cmd in action_map:
                self.master_node.execute_cli_command(action_map[cmd])
                return

        # 2. Stream Enable/Disable & Dynamic Packet Filtering per Node
        target_node = payload.get("node") or nested_data.get("node")

        node_map = {
            "Control Node": "control_tx",
            "3-Axis Control Node": "control_tx",
            "Camera-Stream Node": "camera_tx",
            "Camera Node": "camera_tx",
            "Vision-ArUco Node": "vision_tx",
            "Aruco-Perception Node": "vision_tx",
            "Camera-Aruco Node": "vision_tx",
            "MAVLink Node": "mavlink_tx",
            "PX4-MAVLink Node": "mavlink_tx",
            "Distance-Sensor Node": "distance_tx",
            "Master Node": "master_tx",
            "Master-Logic Node": "master_tx"
        }
        node_key = node_map.get(target_node, target_node)

        if cmd == "STOP_STREAM":
            if node_key:
                self.manually_disabled_nodes.add(node_key)
                if node_key in self.tx_nodes:
                    self.tx_nodes[node_key].pause_stream()
            else:
                self.pause_all_nodes()

        elif cmd == "START_STREAM":
            if node_key:
                self.manually_disabled_nodes.discard(node_key)
                if node_key in self.tx_nodes and self.pc_bridge.is_gcs_connected:
                    self.tx_nodes[node_key].resume_stream()

        elif cmd in ("SELECT_PACKET", "SET_FILTER"):
            packet_name = payload.get("packet") or payload.get("filter") or nested_data.get("packet") or "ALL"
            if node_key in self.tx_nodes and hasattr(self.tx_nodes[node_key], "set_active_filter"):
                self.tx_nodes[node_key].set_active_filter(packet_name)

    def _start_or_update_tx_nodes(self, target_ip: str):
        # 1. Control TX Node (Port 5004)
        if "control_tx" not in self.tx_nodes:
            tx = ControlTransmitter(target_ip=target_ip, port=5004, control_node=self.control_node)
            tx.start()
            tx.set_active_filter("ALL")
            self.tx_nodes["control_tx"] = tx
        else:
            self.tx_nodes["control_tx"].update_target_ip(target_ip)
            if self.control_node:
                self.tx_nodes["control_tx"].control_node = self.control_node

        # 2. Camera Video Stream TX Node (Port 5003)
        if "camera_tx" not in self.tx_nodes:
            cam_tx = CameraTransmitter(target_ip=target_ip, port=5003, camera_node=self.camera_node)
            cam_tx.start()
            cam_tx.set_active_filter("ALL")
            self.tx_nodes["camera_tx"] = cam_tx
        else:
            self.tx_nodes["camera_tx"].update_target_ip(target_ip)
            if self.camera_node:
                self.tx_nodes["camera_tx"].camera_node = self.camera_node

        # 3. MAVLink TX Node (Port 5005)
        if "mavlink_tx" not in self.tx_nodes:
            mav_tx = MavlinkTransmitter(target_ip=target_ip, port=5005, mavlink_node=self.mavlink_node)
            mav_tx.start()
            mav_tx.set_active_filter("ALL")
            self.tx_nodes["mavlink_tx"] = mav_tx
        else:
            self.tx_nodes["mavlink_tx"].update_target_ip(target_ip)
            if self.mavlink_node:
                self.tx_nodes["mavlink_tx"].mavlink_node = self.mavlink_node

        # 4. Distance Sensor TX Node (Port 5002)
        if "distance_tx" not in self.tx_nodes:
            dist_tx = DistanceSensorTransmitter(target_ip=target_ip, port=5002, distance_node=self.distance_node)
            dist_tx.start()
            dist_tx.set_active_filter("ALL")
            self.tx_nodes["distance_tx"] = dist_tx
        else:
            self.tx_nodes["distance_tx"].update_target_ip(target_ip)
            if self.distance_node:
                self.tx_nodes["distance_tx"].distance_node = self.distance_node

        # 5. Master Node TX Node (Port 5001 + Port 5008 CLI Stream)
        if "master_tx" not in self.tx_nodes:
            mst_tx = MasterTransmitter(target_ip=target_ip, port=5001, cli_port=5008, master_node=self.master_node)
            mst_tx.start()
            mst_tx.set_active_filter("ALL")
            self.tx_nodes["master_tx"] = mst_tx
        else:
            self.tx_nodes["master_tx"].update_target_ip(target_ip)
            if self.master_node:
                self.tx_nodes["master_tx"].master_node = self.master_node

        # 6. Vision / ArUco TX Node (Port 5006)
        if "vision_tx" not in self.tx_nodes:
            vis_tx = VisionTransmitter(target_ip=target_ip, port=5006, vision_node=self.vision_node)
            vis_tx.start()
            vis_tx.set_active_filter("ALL")
            self.tx_nodes["vision_tx"] = vis_tx
        else:
            self.tx_nodes["vision_tx"].update_target_ip(target_ip)
            if self.vision_node:
                self.tx_nodes["vision_tx"].vision_node = self.vision_node

    def stop(self):
        self.pc_bridge.stop()
        self.master_cmd_rx.stop()
        for name, node in self.tx_nodes.items():
            if hasattr(node, "stop"):
                node.stop()
        print("[MASTER_TELECOM] All telecom services stopped.")