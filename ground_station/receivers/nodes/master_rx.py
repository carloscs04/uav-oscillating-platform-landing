# ground_station/receivers/nodes/master_rx.py

import json
import socket
import threading
import time
from typing import Dict, Any, Optional, Tuple
from ground_station.comms.pi_bridge import PiLink


class MasterReceiver(threading.Thread):
    """Modular UDP receiver for Master System Node + Remote Console Interface."""

    _active_instance = None

    def __init__(self, host: str = "0.0.0.0", port: int = 5001):
        super().__init__(daemon=True)
        MasterReceiver._active_instance = self

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

    def get_health_info(self) -> Dict[str, Any]:
        with self._lock:
            if self.last_seen == 0.0 or (time.time() - self.last_seen > 2.5) or not self.enabled:
                return {"status": "OFFLINE", "rate_hz": 0.0, "battery_pct": -1, "voltage_v": 0.0}
            info = dict(self.health_info)
            if info.get("status") in ["OFFLINE", "DISCONNECTED", None]:
                info["status"] = "HEALTHY"
            return info

    def send_terminal_command(self, cmd_str: str):
        """Sends CLI command directly to the Master Node on RPi."""
        if self._sock:
            cmd_pkt = json.dumps({
                "cmd": "EXEC_SHELL",
                "node": "Master Node",
                "command": cmd_str
            }).encode("utf-8")
            active_link = getattr(PiLink, "_active_instance", None)
            target_ip = self.last_rpi_ip or (active_link.active_ip if active_link else None)
            if target_ip and target_ip not in ["N/A", "127.0.0.1", "localhost"]:
                try:
                    self._sock.sendto(cmd_pkt, (target_ip, self.port))
                except Exception:
                    pass

    def run(self):
        self.running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(0.5)

        try:
            self._sock.bind((self.host, self.port))
            print(f"[GCS MASTER_RX] Listener bound to {self.host}:{self.port}")
        except Exception as e:
            print(f"[GCS MASTER_RX_ERR] Bind error on port {self.port}: {e}")
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

                    # Extract node_health
                    if "node_health" in incoming_packets:
                        self.health_info = incoming_packets.pop("node_health")

                    # Store dynamic telemetry/terminal packets
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