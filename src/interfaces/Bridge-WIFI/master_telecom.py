# interfaces/Bridge-WIFI/master_telecom.py

import os
import sys
import time
from typing import Dict, Any, Set

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rx_cmd.pc_bridge import PCBridgeListener
from tx_nodes.control_tx import ControlTransmitter
from tx_nodes.camera_tx import CameraTransmitter
from tx_nodes.mavlink_tx import MavlinkTransmitter
from tx_nodes.distance_sensor_tx import DistanceSensorTransmitter


class MasterTelecom:
    """Master Telecommunications Orchestrator for Raspberry Pi.

    Manages connection timeouts, global stream pause/resumes, and per-packet filters.
    """

    def __init__(self, handshake_port: int = 5007):
        self.handshake_port = handshake_port
        self.target_ip: str = None
        self.tx_nodes: Dict[str, Any] = {}
        self.manually_disabled_nodes: Set[str] = set()  # Corrected attribute declaration

        self.pc_bridge = PCBridgeListener(
            port=self.handshake_port,
            timeout_sec=3.0,
            on_ip_discovered=self._on_gcs_connected,
            on_connection_lost=self._on_gcs_disconnected,
            on_command_received=self._on_command_received
        )

    def start(self):
        print("==================================================")
        print("  STARTING RPI MASTER TELECOM (Bridge-WIFI)       ")
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
        """Processes incoming GUI commands (STOP_STREAM, START_STREAM, SELECT_PACKET)."""
        target_node = payload.get("node")

        # Map GUI node display names to internal TX dictionary keys
        node_map = {
            "3-Axis Control Node": "control_tx",
            "Aruco-Perception Node": "camera_tx",
            "PX4-MAVLink Node": "mavlink_tx",
            "Distance-Sensor Node": "distance_tx"
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
        # 1. Control TX Node (Port 5007)
        if "control_tx" not in self.tx_nodes:
            tx = ControlTransmitter(target_ip=target_ip, port=5007)
            tx.start()
            self.tx_nodes["control_tx"] = tx
        else:
            self.tx_nodes["control_tx"].update_target_ip(target_ip)

        # 2. Camera / Perception TX Node (Port 8080)
        if "camera_tx" not in self.tx_nodes:
            cam_tx = CameraTransmitter(target_ip=target_ip, port=8080)
            cam_tx.start()
            self.tx_nodes["camera_tx"] = cam_tx
        else:
            self.tx_nodes["camera_tx"].update_target_ip(target_ip)

        # 3. MAVLink TX Node (Port 5005)
        if "mavlink_tx" not in self.tx_nodes:
            mav_tx = MavlinkTransmitter(target_ip=target_ip, port=5005)
            mav_tx.start()
            self.tx_nodes["mavlink_tx"] = mav_tx
        else:
            self.tx_nodes["mavlink_tx"].update_target_ip(target_ip)

        # 4. Distance Sensor TX Node (Port 5008)
        if "distance_tx" not in self.tx_nodes:
            dist_tx = DistanceSensorTransmitter(target_ip=target_ip, port=5008)
            dist_tx.start()
            self.tx_nodes["distance_tx"] = dist_tx
        else:
            self.tx_nodes["distance_tx"].update_target_ip(target_ip)

    def stop(self):
        self.pc_bridge.stop()
        for name, node in self.tx_nodes.items():
            if hasattr(node, "stop"):
                node.stop()
        print("[MASTER_TELECOM] All telecom services stopped.")


if __name__ == "__main__":
    master = MasterTelecom(handshake_port=5007)
    master.start()