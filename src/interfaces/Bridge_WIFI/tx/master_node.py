# File: src/interfaces/Bridge_WIFI/tx/master_node.py

import json
import socket
import threading
import time
from typing import Any, Dict, Optional


class MasterTransmitter(threading.Thread):
    """Dedicated UDP transmitter for MasterNode FSM status and remote CLI console stream."""

    VALID_NODE_NAMES = {"Master Node", "Master-Logic Node", "MasterNode"}

    def __init__(
        self,
        target_ip: str = "127.0.0.1",
        port: int = 5001,
        cli_port: int = 5008,
        master_node: Optional[Any] = None,
        **kwargs
    ):
        super().__init__(daemon=True)
        self.target_ip = kwargs.get("broadcast_ip", target_ip)
        self.port = port
        self.cli_port = cli_port
        self.master_node = master_node or kwargs.get("master_state")
        self.node_name = kwargs.get("node_name", "Master Node")

        self.running = False
        self.streaming = True
        self.active_filter: str = "ALL"
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def set_active_filter(self, packet_name: str):
        with self._lock:
            self.active_filter = packet_name
            print(f"[MASTER_TX] Active filter set to: '{self.active_filter}'")

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
                time.sleep(0.1)
                continue

            packets_payload = self._build_packets_payload()
            pkt = json.dumps({
                "node": self.node_name,
                "timestamp": time.time(),
                "packets": packets_payload
            }).encode("utf-8")

            try:
                # Port 5001: PiLink telecom health
                self._sock.sendto(pkt, (self.target_ip, self.port))
                # Port 5008: Dedicated MasterReceiver & cmd_master console stream
                self._sock.sendto(pkt, (self.target_ip, self.cli_port))
            except Exception as e:
                print(f"[MASTER_TX_ERR] Send error to {self.target_ip} -> {e}")

            time.sleep(0.1)

        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

    def _build_packets_payload(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {}

        with self._lock:
            node_ref = self.master_node
            if node_ref:
                try:
                    state = getattr(node_ref, "state", node_ref)
                    is_connected = getattr(state, "connected", True)
                    rate_hz = getattr(state, "current_rate_hz", 10.0)

                    if hasattr(state, "export_node_health"):
                        payload["node_health"] = state.export_node_health(rate_hz, is_connected)

                    if hasattr(state, "export_telemetry_data"):
                        telemetry_data = state.export_telemetry_data()
                        if isinstance(telemetry_data, dict):
                            for pkt_name, pkt_data in telemetry_data.items():
                                if self.active_filter == "ALL" or pkt_name in ("cli_stream", "master_status", self.active_filter):
                                    payload[pkt_name] = pkt_data

                    if payload:
                        return payload
                except Exception as e:
                    print(f"[MASTER_TX_ERR] Failed to extract MasterState data: {e}")

            payload["node_health"] = {
                "status": "OFFLINE",
                "rate_hz": 0.0,
                "battery_pct": -1,
                "voltage_v": 0.0
            }

        return payload

    def stop(self):
        self.running = False
        self.streaming = False