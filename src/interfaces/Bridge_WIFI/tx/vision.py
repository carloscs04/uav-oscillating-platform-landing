# File: src/interfaces/Bridge_WIFI/tx/vision.py

import json
import socket
import threading
import time
from typing import Any, Dict, Optional


class VisionTransmitter(threading.Thread):
    """Dedicated UDP transmitter for ArUco marker detection and pose telemetry."""

    VALID_NODE_NAMES = {"Vision-ArUco Node", "Aruco-Perception Node", "Vision Node"}

    def __init__(self, target_ip: str = "127.0.0.1", port: int = 5006, vision_node: Optional[Any] = None, **kwargs):
        super().__init__(daemon=True)
        self.target_ip = kwargs.get("broadcast_ip", target_ip)
        self.port = port
        self.vision_node = vision_node or kwargs.get("vision_state")
        self.node_name = kwargs.get("node_name", "Vision-ArUco Node")

        self.running = False
        self.streaming = True
        self.active_filter: str = "ALL"
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def process_command(self, command: Dict[str, Any]):
        """Handle incoming GCS commands for packet filtering and stream state."""
        target_node = command.get("node")
        legacy_node = command.get("legacy_node")

        if target_node in self.VALID_NODE_NAMES or legacy_node in self.VALID_NODE_NAMES:
            cmd_type = command.get("cmd") or command.get("command")
            new_filter = command.get("filter") or command.get("packet")
            if new_filter:
                self.set_active_filter(new_filter)

            if cmd_type == "START_STREAM":
                self.resume_stream()
            elif cmd_type == "STOP_STREAM":
                self.pause_stream()

    def set_active_filter(self, packet_name: str):
        with self._lock:
            self.active_filter = packet_name
            print(f"[VISION_TX] Active filter set to: '{self.active_filter}'")

    def update_target_ip(self, new_ip: str):
        self.target_ip = new_ip

    def pause_stream(self):
        self.streaming = False

    def resume_stream(self):
        self.streaming = True

    def run(self):
        self.running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        while self.running:
            if not self.target_ip or not self.streaming:
                time.sleep(0.05)
                continue

            packets_payload = self._build_packets_payload()
            pkt = json.dumps({
                "node": self.node_name,
                "timestamp": time.time(),
                "packets": packets_payload
            }).encode("utf-8")

            try:
                self._sock.sendto(pkt, (self.target_ip, self.port))
            except Exception as e:
                print(f"[VISION_TX_ERR] Send error to {self.target_ip}:{self.port} -> {e}")

            # 30 Hz high-priority pose loop
            time.sleep(0.033)

        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

    def _build_packets_payload(self) -> Dict[str, Any]:
        """Pull ArUco telemetry and health status from VisionState."""
        payload: Dict[str, Any] = {}

        with self._lock:
            node_ref = self.vision_node
            if node_ref:
                try:
                    state = getattr(node_ref, "state", node_ref)
                    is_connected = getattr(node_ref, "connected", getattr(state, "connected", True))
                    rate_hz = getattr(node_ref, "current_rate_hz", getattr(state, "current_rate_hz", 30.0))

                    if hasattr(state, "export_node_health"):
                        payload["node_health"] = state.export_node_health(rate_hz, is_connected)

                    if hasattr(state, "export_telemetry_data"):
                        telemetry_data = state.export_telemetry_data()
                        if isinstance(telemetry_data, dict):
                            for pkt_name, pkt_data in telemetry_data.items():
                                if self.active_filter == "ALL" or self.active_filter == pkt_name:
                                    payload[pkt_name] = pkt_data

                    if payload:
                        return payload
                except Exception as e:
                    print(f"[VISION_TX_ERR] Failed to extract state data: {e}")

            payload["node_health"] = {
                "status": "OFFLINE",
                "rate_hz": 0.0,
                "battery_pct": -1,
                "voltage_v": 0.0
            }
            if self.active_filter != "ALL":
                payload[self.active_filter] = {"status": "NO_CONNECTION_ESTABLISHED"}
            else:
                payload["status"] = "NO_CONNECTION_ESTABLISHED"

        return payload

    def stop(self):
        self.running = False
        self.streaming = False