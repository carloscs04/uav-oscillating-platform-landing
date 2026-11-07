# interfaces/Bridge-WIFI/tx_nodes/mavlink_tx.py

import json
import math
import socket
import threading
import time
from typing import Any, Callable, Dict, Optional


class MavlinkTransmitter(threading.Thread):
    """Modular UDP transmitter for PX4 MAVLink Telemetry."""

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

        self.running = False
        self.streaming = True
        self.active_filter: str = "velocity"
        self._sock: Optional[socket.socket] = None

        self._packet_providers: Dict[str, Callable[[], Any]] = {}
        self._static_packets: Dict[str, Any] = {}
        self._lock = threading.Lock()

        self._setup_default_providers()

    def set_active_filter(self, packet_name: str):
        with self._lock:
            self.active_filter = packet_name
            print(f"[MAVLINK_TX] Active filter set to: '{self.active_filter}'")

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
                "node": "PX4-MAVLink Node",
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
        payload = {}
        with self._lock:
            payload["node_health"] = self._get_health_status()

            for pkt_type, provider in self._packet_providers.items():
                if self.active_filter == "ALL" or self.active_filter == pkt_type:
                    try:
                        payload[pkt_type] = provider()
                    except Exception as e:
                        print(f"[MAVLINK_TX_ERR] Provider '{pkt_type}' failed: {e}")

            for pkt_type, data in self._static_packets.items():
                if (self.active_filter == "ALL" or self.active_filter == pkt_type) and pkt_type not in payload:
                    payload[pkt_type] = data
        return payload

    def _get_health_status(self) -> Dict[str, Any]:
        return {"status": "HEALTHY", "mavlink_connected": True}

    def _setup_default_providers(self):
        def _gen_velocity():
            # -----------------------------------------------------------------
            # REAL HARDWARE NODE CONNECTION (Uncomment when ready)
            # -----------------------------------------------------------------
            if self.mavlink_node and hasattr(self.mavlink_node, "get_velocity"):
                return self.mavlink_node.get_velocity()

            # =================================================================
            # SIMULATION FALLBACK BLOCK (EASY TO REMOVE / COMMENT OUT)
            # =================================================================
            now = time.time()
            return {
                "vx": round(0.5 * math.cos(now * 0.5), 3),
                "vy": round(0.5 * math.sin(now * 0.5), 3),
                "vz": round(0.05 * math.sin(now * 2.0), 3)
            }
            # =================================================================

        def _gen_distance_sensor():
            # -----------------------------------------------------------------
            # REAL HARDWARE NODE CONNECTION (Uncomment when ready)
            # -----------------------------------------------------------------
            if self.mavlink_node and hasattr(self.mavlink_node, "get_distance_sensor"):
                return self.mavlink_node.get_distance_sensor()

            # =================================================================
            # SIMULATION FALLBACK BLOCK (EASY TO REMOVE / COMMENT OUT)
            # =================================================================
            now = time.time()
            return {
                "distance": round(1.50 + 0.1 * math.sin(now * 1.5), 3),
                "min_distance": 0.20,
                "max_distance": 4.00
            }
            # =================================================================

        self.register_packet_provider("velocity", _gen_velocity)
        self.register_packet_provider("distance_sensor", _gen_distance_sensor)

    def stop(self):
        self.running = False
        self.streaming = False