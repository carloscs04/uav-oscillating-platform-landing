# ground_station/receivers/nodes/distance_sensor_rx.py

import json
import socket
import threading
import time
from typing import Dict, Any, Optional, Tuple

from ground_station.comms.pi_bridge import PiLink, LinkState


class DistanceSensorReceiver(threading.Thread):
    """Modular UDP receiver for Standalone Rangefinder Telemetry."""

    _active_instance = None

    def __init__(self, host: str = "0.0.0.0", port: int = 5008):
        super().__init__(daemon=True)
        DistanceSensorReceiver._active_instance = self

        self.host = host
        self.port = port
        self.running = False
        self.enabled = True

        self._lock = threading.Lock()
        self.rx_state = "DISCONNECTED"
        self.last_seen: float = 0.0

        # Pre-initialize packet repository for immediate GUI tab rendering
        self.packets: Dict[str, Any] = {
            "rangefinder": {"distance": 1.85, "min_range": 0.2, "max_range": 6.0, "signal_quality": 98},
            "sensor_health": {"status": "OK", "voltage": 5.01}
        }
        self.distance: float = 1.85
        self.rangefinder: Dict[str, Any] = {"distance": 1.85, "signal_quality": 98}
        self.sensor_health: Dict[str, Any] = {"status": "OK", "voltage": 5.01}

        self.rpi_address: Optional[Tuple[str, int]] = None
        self.last_rpi_ip: Optional[str] = None
        self._sock: Optional[socket.socket] = None

    def get_packet(self, pkt_type: str, default: Any = None) -> Any:
        with self._lock:
            return self.packets.get(pkt_type, default)

    def get_distance(self) -> float:
        with self._lock:
            return self.distance

    def get_status_info(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "status": self.rx_state,
                "port": self.port,
                "rate": 20,
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
                "node": "Distance-Sensor Node"
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
            print(f"[GCS DISTANCE_RX] Listener bound to 0.0.0.0:{self.port}")
        except Exception as e:
            print(f"[GCS DISTANCE_RX_ERR] Bind error on port {self.port}: {e}")
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
                data, addr = self._sock.recvfrom(4096)
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
                        if pkt_type == "rangefinder":
                            self.rangefinder = pkt_data
                            self.distance = pkt_data.get("distance", self.distance)
                        elif pkt_type == "sensor_health":
                            self.sensor_health = pkt_data

            except socket.timeout:
                continue
            except Exception:
                if self.running:
                    time.sleep(0.05)

        if self._sock:
            self._sock.close()

    def stop(self):
        self.running = False