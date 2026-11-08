import json
import socket
import threading
import time
from typing import Any, Dict, Optional


class ControlTransmitter(threading.Thread):
    """Modular UDP transmitter for 3-Axis Trajectory Control telemetry."""

    def __init__(
        self,
        target_ip: str = "127.0.0.1",
        port: int = 5007,
        control_node: Optional[Any] = None,
        **kwargs
    ):
        super().__init__(daemon=True)
        self.target_ip = kwargs.get("broadcast_ip", target_ip)
        self.port = port
        self.control_node = control_node

        self.running = False
        self.streaming = True
        self.active_filter: str = "ALL"
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def set_active_filter(self, packet_name: str):
        with self._lock:
            self.active_filter = packet_name

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
                "node": "3-Axis Control Node",
                "timestamp": time.time(),
                "packets": packets_payload
            }).encode("utf-8")

            try:
                self._sock.sendto(pkt, (self.target_ip, self.port))
            except Exception as e:
                print(f"[CONTROL_TX_ERR] Send error to {self.target_ip}: {e}")

            time.sleep(0.05)

        if self._sock:
            self._sock.close()

    def _build_packets_payload(self, elapsed: float) -> Dict[str, Any]:
        """Pulls pre-structured control packages directly from Control Node."""
        payload = {}
        with self._lock:
            payload["node_health"] = self._get_health_status()

            if self.control_node and hasattr(self.control_node, "export_telemetry_package"):
                try:
                    packages = self.control_node.export_telemetry_package()
                    if isinstance(packages, dict):
                        for pkt_name, pkt_data in packages.items():
                            if self.active_filter == "ALL" or self.active_filter == pkt_name:
                                payload[pkt_name] = pkt_data
                        return payload
                except Exception as e:
                    print(f"[CONTROL_TX_ERR] Failed to pull telemetry package: {e}")

            if self.active_filter != "ALL":
                payload[self.active_filter] = {"status": "NO_CONNECTION_ESTABLISHED"}
            else:
                payload["status"] = "NO_CONNECTION_ESTABLISHED"

        return payload

    def _get_health_status(self) -> Dict[str, Any]:
        is_connected = bool(self.control_node is not None)
        return {"status": "HEALTHY" if is_connected else "NO_CONNECTION_ESTABLISHED", "connected": is_connected}

    def stop(self):
        self.running = False
        self.streaming = False