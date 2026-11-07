# interfaces/Bridge-WIFI/tx_nodes/control_tx.py

import json
import math
import socket
import threading
import time
from typing import Any, Callable, Dict, Optional


class ControlTransmitter(threading.Thread):
    """Plug-and-play UDP transmitter for 3-Axis Control Telemetry."""

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
        self.active_filter: str = "trajectory"
        self._sock: Optional[socket.socket] = None

        self._packet_providers: Dict[str, Callable[[], Any]] = {}
        self._static_packets: Dict[str, Any] = {}
        self._lock = threading.Lock()

        self._setup_default_providers()

    def set_active_filter(self, packet_name: str):
        """Sets the active inspection packet filter."""
        with self._lock:
            self.active_filter = packet_name
            print(f"[CONTROL_TX] Active terminal filter set to: '{self.active_filter}'")

    def set_packet(self, pkt_type: str, data: Any):
        with self._lock:
            self._static_packets[pkt_type] = data

    def register_packet_provider(self, pkt_type: str, provider: Callable[[], Any]):
        with self._lock:
            self._packet_providers[pkt_type] = provider

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
        payload = {}
        with self._lock:
            payload["node_health"] = self._get_health_status()

            # Mandatory trajectory stream to keep live plotting canvases active
            if "trajectory" in self._packet_providers:
                try:
                    payload["trajectory"] = self._packet_providers["trajectory"]()
                except Exception as e:
                    print(f"[CONTROL_TX_ERR] Trajectory provider error: {e}")

            # Append dynamic inspection packet if requested
            target_pkt = self.active_filter
            if target_pkt != "trajectory" and target_pkt != "ALL":
                if target_pkt in self._packet_providers:
                    try:
                        payload[target_pkt] = self._packet_providers[target_pkt]()
                    except Exception as e:
                        print(f"[CONTROL_TX_ERR] Provider '{target_pkt}' error: {e}")
                elif target_pkt in self._static_packets:
                    payload[target_pkt] = self._static_packets[target_pkt]

        return payload

    def _get_health_status(self) -> Dict[str, Any]:
        return {"status": "HEALTHY", "cpu_load_pct": 12.4, "temp_c": 44.2, "error_count": 0}

    def _setup_default_providers(self):
        def _gen_trajectory():
            if self.control_node and hasattr(self.control_node, "get_trajectory"):
                return self.control_node.get_trajectory()

            # Simulation Fallback
            now = time.time()
            ref = [round(math.sin(now * 0.8), 3), round(math.cos(now * 0.8), 3), 1.5]
            act = [round(ref[0] * 0.95, 3), round(ref[1] * 0.95, 3), round(ref[2] - 0.03, 3)]
            return {"ref": ref, "act": act}

        def _gen_io_data():
            if self.control_node and hasattr(self.control_node, "get_io_data"):
                return self.control_node.get_io_data()

            # Simulation Fallback
            return {"inputs": [0.0, 0.0, 0.0], "outputs": [1500, 1500, 1500, 1500]}

        def _gen_errors():
            if self.control_node and hasattr(self.control_node, "get_errors"):
                return self.control_node.get_errors()

            # Simulation Fallback
            return {"err": [0.01, -0.02, 0.005]}

        self.register_packet_provider("trajectory", _gen_trajectory)
        self.register_packet_provider("io_data", _gen_io_data)
        self.register_packet_provider("errors", _gen_errors)

    def stop(self):
        self.running = False
        self.streaming = False