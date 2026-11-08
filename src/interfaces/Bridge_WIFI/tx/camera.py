# interfaces/Bridge-WIFI/tx_nodes/camera_tx.py

import json
import math
import socket
import threading
import time
from typing import Any, Callable, Dict, Optional


class CameraTransmitter(threading.Thread):
    """Modular UDP transmitter for Perception & Video with dynamic packet filtering."""

    def __init__(
        self,
        target_ip: str = "127.0.0.1",
        port: int = 8080,
        camera_node: Optional[Any] = None,
        **kwargs
    ):
        super().__init__(daemon=True)
        self.target_ip = kwargs.get("broadcast_ip", target_ip)
        self.port = port
        self.camera_node = camera_node

        self.running = False
        self.streaming = True
        self.active_filter: str = "ALL"  # 'ALL' or specific packet name
        self._sock: Optional[socket.socket] = None

        self._packet_providers: Dict[str, Callable[[], Any]] = {}
        self._static_packets: Dict[str, Any] = {}
        self._lock = threading.Lock()

        self._setup_default_providers()

    def set_active_filter(self, packet_name: str):
        """Filters stream output to transmit ONLY the selected packet or 'ALL'."""
        with self._lock:
            self.active_filter = packet_name
            print(f"[CAMERA_TX] Active filter set to: '{self.active_filter}'")

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
                "node": "Aruco-Perception Node",
                "timestamp": time.time(),
                "packets": packets_payload
            }).encode("utf-8")

            try:
                self._sock.sendto(pkt, (self.target_ip, self.port))
            except Exception as e:
                print(f"[CAMERA_TX_ERR] Send error to {self.target_ip}: {e}")

            time.sleep(0.066)  # ~15 FPS

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
                        print(f"[CAMERA_TX_ERR] Provider '{pkt_type}' failed: {e}")

            for pkt_type, data in self._static_packets.items():
                if (self.active_filter == "ALL" or self.active_filter == pkt_type) and pkt_type not in payload:
                    payload[pkt_type] = data
        return payload

    def _get_health_status(self) -> Dict[str, Any]:
        return {"status": "HEALTHY", "fps": 15.0, "error_count": 0}

    def _setup_default_providers(self):
        def _gen_aruco_pose():
            if self.camera_node and hasattr(self.camera_node, "get_aruco_pose"):
                return self.camera_node.get_aruco_pose()
            
            now = time.time()
            return {
                "id": 42,
                "detected": True,
                "x": round(0.15 * math.sin(now), 3),
                "y": round(0.10 * math.cos(now), 3),
                "z": round(1.25 + 0.05 * math.sin(now * 2.0), 3)
            }

        def _gen_video_frame():
            if self.camera_node and hasattr(self.camera_node, "get_encoded_frame"):
                return self.camera_node.get_encoded_frame()
            return {"format": "jpeg_b64", "data": "", "resolution": [640, 480]}

        self.register_packet_provider("aruco_pose", _gen_aruco_pose)
        self.register_packet_provider("video_frame", _gen_video_frame)

    def stop(self):
        self.running = False
        self.streaming = False