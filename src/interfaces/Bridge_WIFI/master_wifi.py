# File: src/interfaces/Bridge_WIFI/master_wifi.py

import os
import sys
import time
from typing import Dict, Any, Set, Optional

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rx.pc_bridge import PCBridgeListener
from tx.control_node import ControlTransmitter
from tx.camera import CameraTransmitter
from tx.mavlink import MavlinkTransmitter
from tx.distance_sensor import DistanceSensorTransmitter
from tx.master_node import MasterTransmitter


class MasterWIFI:
    """Master Telecommunications Orchestrator for Raspberry Pi.

    Manages connection timeouts, global stream pause/resumes, per-packet filters,
    and forwards live telemetry packages from all onboard subsystem nodes over UDP.
    """

    def __init__(
        self,
        handshake_port: int = 5007,
        mavlink_node: Optional[Any] = None,
        control_node: Optional[Any] = None,
        camera_node: Optional[Any] = None,
        distance_node: Optional[Any] = None,
        master_node: Optional[Any] = None
    ):
        self.handshake_port = handshake_port
        
        # Subsystem Node References
        self.mavlink_node = mavlink_node
        self.control_node = control_node
        self.camera_node = camera_node
        self.distance_node = distance_node
        self.master_node = master_node

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

    def set_mavlink_node(self, node: Any):
        """Dynamically attach or update the live MAVLink node reference."""
        self.mavlink_node = node
        if "mavlink_tx" in self.tx_nodes:
            self.tx_nodes["mavlink_tx"].mavlink_node = node

    def set_control_node(self, node: Any):
        """Dynamically attach or update the live 3-Axis Control node reference."""
        self.control_node = node
        if "control_tx" in self.tx_nodes:
            self.tx_nodes["control_tx"].control_node = node

    def set_camera_node(self, node: Any):
        """Dynamically attach or update the live Camera / Perception node reference."""
        self.camera_node = node
        if "camera_tx" in self.tx_nodes:
            self.tx_nodes["camera_tx"].camera_node = node

    def set_distance_node(self, node: Any):
        """Dynamically attach or update the live Distance Sensor node reference."""
        self.distance_node = node
        if "distance_tx" in self.tx_nodes:
            self.tx_nodes["distance_tx"].distance_node = node

    def set_master_node(self, node: Any):
        """Dynamically attach or update the live Master Node reference."""
        self.master_node = node
        if "master_tx" in self.tx_nodes:
            self.tx_nodes["master_tx"].master_node = node

    def start(self):
        print("==================================================")
        print("   STARTING RPI MASTER TELECOM (Bridge-WIFI)      ")
        print("==================================================")

        self.pc_bridge.start()
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
        target_node = payload.get("node")

        node_map = {
            "3-Axis Control Node": "control_tx",
            "Aruco-Perception Node": "camera_tx",
            "PX4-MAVLink Node": "mavlink_tx",
            "Distance-Sensor Node": "distance_tx",
            "Master Node": "master_tx",
            "Master-Logic Node": "master_tx"
        }
        node_key = node_map.get(target_node, target_node)

        if cmd == "STOP_STREAM":
            print(f"[MASTER_TELECOM] Command 'STOP_STREAM' for node: {target_node or 'ALL'}")
            if node_key:
                self.manually_disabled_nodes.add(node_key)
                if node_key in self.tx_nodes:
                    self.tx_nodes[node_key].pause_stream()
            else:
                self.pause_all_nodes()

        elif cmd == "START_STREAM":
            print(f"[MASTER_TELECOM] Command 'START_STREAM' for node: {target_node or 'ALL'}")
            if node_key:
                self.manually_disabled_nodes.discard(node_key)
                if node_key in self.tx_nodes and self.pc_bridge.is_gcs_connected:
                    self.tx_nodes[node_key].resume_stream()

        elif cmd == "SELECT_PACKET":
            packet_name = payload.get("packet", "ALL")
            print(f"[MASTER_TELECOM] Dynamic filter '{packet_name}' requested for node: {node_key}")
            if node_key in self.tx_nodes and hasattr(self.tx_nodes[node_key], "set_active_filter"):
                self.tx_nodes[node_key].set_active_filter(packet_name)

    def _start_or_update_tx_nodes(self, target_ip: str):
        # 1. Control TX Node (Port 5004 / 5007)
        if "control_tx" not in self.tx_nodes:
            tx = ControlTransmitter(target_ip=target_ip, port=5004, control_node=self.control_node)
            tx.start()
            tx.set_active_filter("ALL")
            self.tx_nodes["control_tx"] = tx
        else:
            self.tx_nodes["control_tx"].update_target_ip(target_ip)
            if self.control_node:
                self.tx_nodes["control_tx"].control_node = self.control_node

        # 2. Camera / Perception TX Node (Port 5003)
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

        # 5. Master Node TX Node (Port 5001)
        if "master_tx" not in self.tx_nodes:
            mst_tx = MasterTransmitter(target_ip=target_ip, port=5001, master_node=self.master_node)
            mst_tx.start()
            mst_tx.set_active_filter("ALL")
            self.tx_nodes["master_tx"] = mst_tx
        else:
            self.tx_nodes["master_tx"].update_target_ip(target_ip)
            if self.master_node:
                self.tx_nodes["master_tx"].master_node = self.master_node

    def stop(self):
        self.pc_bridge.stop()
        for name, node in self.tx_nodes.items():
            if hasattr(node, "stop"):
                node.stop()
        print("[MASTER_TELECOM] All telecom services stopped.")


if __name__ == "__main__":
    master = MasterWIFI(handshake_port=5007)
    master.start()