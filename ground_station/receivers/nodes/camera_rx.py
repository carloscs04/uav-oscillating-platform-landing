# ground_station/receivers/nodes/camera_rx.py

import json
import math
import socket
import threading
import time
from typing import Dict, Any, Optional, Tuple

from ground_station.comms.pi_bridge import PiLink, LinkState


class CameraReceiver(threading.Thread):
    """Modular UDP receiver for Vision & ArUco Perception Telemetry."""

    _active_instance = None

    def __init__(self, host: str = "0.0.0.0", port: int = 8080):
        super().__init__(daemon=True)
        CameraReceiver._active_instance = self

        self.host = host
        self.port = port
        self.running = False
        self.enabled = True

        self._lock = threading.Lock()
        self.rx_state = "DISCONNECTED"
        self.last_seen: float = 0.0

        # Pre-initialize packet repository for immediate GUI tab rendering
        self.packets: Dict[str, Any] = {
            "aruco_pose": {"id": 42, "detected": True, "x": 0.0, "y": 0.0, "z": 1.25},
            "video_frame": {"format": "jpeg_b64", "data": "", "resolution": [640, 480]}
        }
        self.aruco_pose: Dict[str, Any] = {"id": 42, "detected": True, "x": 0.0, "y": 0.0, "z": 1.25}

        self.rpi_address: Optional[Tuple[str, int]] = None
        self.last_rpi_ip: Optional[str] = None
        self._sock: Optional[socket.socket] = None

    def get_packet(self, pkt_type: str, default: Any = None) -> Any:
        with self._lock:
            return self.packets.get(pkt_type, default)

    def get_aruco_distance(self) -> float:
        with self._lock:
            x = self.aruco_pose.get("x", 0.0)
            y = self.aruco_pose.get("y", 0.0)
            z = self.aruco_pose.get("z", 0.0)
            return round(math.sqrt(x**2 + y**2 + z**2), 2)

    def get_status_info(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "status": self.rx_state,
                "port": self.port,
                "rate": 15,
                "telemetry": dict(self.packets),
                "last_seen": self.last_seen
            }

    def _is_pi_link_connected(self) -> bool:
        if not self.enabled:
            return False
        active_link = getattr(PiLink, "_active_instance", None)
        if active_link is None:
            return False
        try:
            return active_link.link_state == LinkState.CONNECTED
        except Exception:
            return False

    def _send_start_signal(self):
        if self._sock:
            start_pkt = json.dumps({
                "cmd": "START_STREAM",
                "node": "Aruco-Perception Node"
            }).encode("utf-8")
            active_link = getattr(PiLink, "_active_instance", None)
            target_ip = self.last_rpi_ip or (active_link.active_ip if active_link else None)
            if target_ip and target_ip not in ["N/A", "127.0.0.1", "localhost"]:
                try:
                    self._sock.sendto(start_pkt, (target_ip, self.port))
                except Exception:
                    pass

    def run(self):
        self.running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(0.5)

        try:
            self._sock.bind(("0.0.0.0", self.port))
            print(f"[GCS CAMERA_RX] Listener bound to 0.0.0.0:{self.port}")
        except Exception as e:
            print(f"[GCS CAMERA_RX_ERR] Bind error on port {self.port}: {e}")
            return

        while self.running:
            is_connected = self._is_pi_link_connected()

            if not is_connected:
                with self._lock:
                    self.rx_state = "DISABLED" if not self.enabled else "DISCONNECTED"
                time.sleep(0.2)
                continue

            with self._lock:
                if self.rx_state in ["DISCONNECTED", "DISABLED"]:
                    self.rx_state = "STREAMING"
                    self._send_start_signal()

            try:
                data, addr = self._sock.recvfrom(65535)
                if not data:
                    continue

                payload = json.loads(data.decode("utf-8"))
                self.rpi_address = addr
                self.last_rpi_ip = addr[0]

                with self._lock:
                    self.rx_state = "STREAMING"
                    self.last_seen = time.time()
                    incoming_packets = payload.get("packets", {})

                    for pkt_type, pkt_data in incoming_packets.items():
                        self.packets[pkt_type] = pkt_data
                        if pkt_type == "aruco_pose":
                            self.aruco_pose = pkt_data

            except socket.timeout:
                continue
            except Exception:
                if self.running:
                    time.sleep(0.05)

        if self._sock:
            self._sock.close()

    def stop(self):
        self.running = False