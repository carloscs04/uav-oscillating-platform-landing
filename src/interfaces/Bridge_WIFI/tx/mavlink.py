import json
import socket
import threading
import time
from typing import Any, Dict, Optional


class MavlinkTransmitter(threading.Thread):
    """Modular UDP transmitter for PX4 MAVLink Telemetry."""

    # Supported node alias identifiers for incoming command matching
    VALID_NODE_NAMES = {"MAVLink Node", "PX4-MAVLink Node"}

    def __init__(
        self,
        target_ip: str = "127.0.0.1",
        port: int = 5005,
        mavlink_node: Optional[Any] = None,
        **kwargs
    ):
        super().__init__(daemon=True)
        self.target_ip = kwargs.get("broadcast_ip", target_ip)
        self.port = port
        self.mavlink_node = mavlink_node
        self.node_name = kwargs.get("node_name", "MAVLink Node")

        self.running = False
        self.streaming = True
        self.active_filter: str = "ALL"
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def process_command(self, command: Dict[str, Any]):
        """Processes incoming GCS control commands (e.g., SET_FILTER, START_STREAM, SELECT_PACKET)."""
        target_node = command.get("node")
        legacy_node = command.get("legacy_node")

        # Validate target node against supported alias identifiers
        if target_node in self.VALID_NODE_NAMES or legacy_node in self.VALID_NODE_NAMES:
            cmd_type = command.get("cmd") or command.get("command")

            # Update telemetry packet filter selection
            new_filter = command.get("filter") or command.get("packet")
            if new_filter:
                self.set_active_filter(new_filter)

            # Handle stream activation toggles
            if cmd_type == "START_STREAM":
                self.resume_stream()
            elif cmd_type == "STOP_STREAM":
                self.pause_stream()

    def set_active_filter(self, packet_name: str):
        with self._lock:
            self.active_filter = packet_name
            print(f"[MAVLINK_TX] Active filter set to: '{self.active_filter}'")

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

        t_start = time.time()

        while self.running:
            if not self.target_ip or not self.streaming:
                time.sleep(0.1)
                continue

            elapsed = time.time() - t_start
            packets_payload = self._build_packets_payload(elapsed)

            pkt = json.dumps({
                "node": self.node_name,
                "timestamp": time.time(),
                "packets": packets_payload
            }).encode("utf-8")

            try:
                self._sock.sendto(pkt, (self.target_ip, self.port))
            except Exception as e:
                print(f"[MAVLINK_TX_ERR] Send error to {self.target_ip}: {e}")

            time.sleep(0.05)

        if self._sock:
            self._sock.close()

    def _build_packets_payload(self, elapsed: float) -> Dict[str, Any]:
        """Pulls telemetry packages from MAVLink Node and ensures node_health always bypasses filters."""
        payload = {}

        with self._lock:
            if self.mavlink_node and hasattr(self.mavlink_node, "export_telemetry_package"):
                try:
                    telemetry_packages = self.mavlink_node.export_telemetry_package()
                    if isinstance(telemetry_packages, dict):
                        # 1. Always extract and include node_health (bypassing dynamic active_filter)
                        if "node_health" in telemetry_packages:
                            payload["node_health"] = telemetry_packages.pop("node_health")

                        # 2. Filter remaining telemetry packages (velocity, attitude, etc.) based on active_filter
                        for pkt_name, pkt_data in telemetry_packages.items():
                            if self.active_filter == "ALL" or self.active_filter == pkt_name:
                                payload[pkt_name] = pkt_data

                        return payload
                except Exception as e:
                    print(f"[MAVLINK_TX_ERR] Failed to pull telemetry package: {e}")

            # Fallback health status if node is disconnected or unavailable
            payload["node_health"] = {
                "status": "RECONNECTING",
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