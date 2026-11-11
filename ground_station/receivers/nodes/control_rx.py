# File: ground_station/receivers/nodes/control_rx.py

import json
import socket
import threading
import time
from typing import Dict, Any, Optional, Tuple, List


class ControlReceiver(threading.Thread):
    """Modular UDP receiver for Control & Vision ArUco Pose telemetry."""

    _active_instance = None

    def __init__(self, host: str = "0.0.0.0", port: int = 5004):
        super().__init__(daemon=True)
        ControlReceiver._active_instance = self

        self.host = host
        self.port = port
        self.running = False
        self.enabled = True

        self._lock = threading.Lock()
        self.rx_state = "DISCONNECTED"
        self.last_seen: float = 0.0

        self.packets: Dict[str, Any] = {}
        self.health_info: Dict[str, Any] = {
            "status": "OFFLINE",
            "rate_hz": 0.0,
            "battery_pct": -1,
            "voltage_v": 0.0
        }

        self.rpi_address: Optional[Tuple[str, int]] = None
        self.last_rpi_ip: Optional[str] = None
        self._sock: Optional[socket.socket] = None

    def get_packet(self, pkt_type: str, default: Any = None) -> Any:
        with self._lock:
            return self.packets.get(pkt_type, default)

    def get_all_packets(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self.packets)

    def get_control_data(self) -> Tuple[List[float], List[float]]:
        """Extracts pos_ref and pos_act safely for ControlView3Axis."""
        with self._lock:
            # 1. Intenta extraer del paquete 'control_state'
            ctrl = self.packets.get("control_state", {})
            pos_ref = ctrl.get("pos_ref")
            pos_act = ctrl.get("pos_act")

            # 2. Fallback a 'pid_output' si 'control_state' no está definido
            if pos_ref is None or pos_act is None:
                pid = self.packets.get("pid_output", {})
                pos_ref = pos_ref or pid.get("pos_ref", [0.0, 0.0, 1.5])
                pos_act = pos_act or pid.get("pos_act", [0.0, 0.0, 0.0])

            # 3. Fallback a valores por defecto
            if not isinstance(pos_ref, list) or len(pos_ref) < 3:
                pos_ref = [0.0, 0.0, 1.5]
            if not isinstance(pos_act, list) or len(pos_act) < 3:
                pos_act = [0.0, 0.0, 0.0]

            return pos_ref, pos_act

    def get_health_info(self) -> Dict[str, Any]:
        with self._lock:
            if self.last_seen == 0.0 or (time.time() - self.last_seen > 2.5) or not self.enabled:
                return {"status": "OFFLINE", "rate_hz": 0.0, "battery_pct": -1, "voltage_v": 0.0}
            info = dict(self.health_info)
            if info.get("status") in ["OFFLINE", "DISCONNECTED", None]:
                info["status"] = "HEALTHY"
            return info

    def get_status_info(self) -> Dict[str, Any]:
        with self._lock:
            is_active = (time.time() - self.last_seen <= 2.5) and self.enabled
            return {
                "status": "STREAMING" if is_active else "OFFLINE",
                "port": self.port,
                "rate": self.health_info.get("rate_hz", 0.0),
                "telemetry": dict(self.packets),
                "last_seen": self.last_seen
            }

    def run(self):
        self.running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(0.5)

        try:
            self._sock.bind((self.host, self.port))
            print(f"[GCS CONTROL_RX] Listener bound to {self.host}:{self.port}")
        except Exception as e:
            print(f"[GCS CONTROL_RX_ERR] Bind error on port {self.port}: {e}")
            return

        while self.running:
            if not self.enabled:
                with self._lock:
                    self.rx_state = "DISABLED"
                time.sleep(0.2)
                continue

            try:
                data, addr = self._sock.recvfrom(4096)
                if not data:
                    continue

                payload = json.loads(data.decode("utf-8"))
                if not isinstance(payload, dict) or "packets" not in payload:
                    continue

                self.rpi_address = addr
                self.last_rpi_ip = addr[0]

                with self._lock:
                    self.rx_state = "STREAMING"
                    self.last_seen = time.time()
                    incoming_packets = payload["packets"]

                    if "node_health" in incoming_packets:
                        self.health_info = incoming_packets.pop("node_health")

                    for pkt_type, pkt_data in incoming_packets.items():
                        self.packets[pkt_type] = pkt_data

            except socket.timeout:
                continue
            except Exception:
                if self.running:
                    time.sleep(0.05)

        if self._sock:
            self._sock.close()

    def stop(self):
        self.running = False