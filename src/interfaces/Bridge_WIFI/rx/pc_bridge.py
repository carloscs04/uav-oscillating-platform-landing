# interfaces/Bridge-WIFI/rx/pc_bridge.py

import json
import socket
import threading
import time
from typing import Callable, Optional, Dict, Any


class PCBridgeListener(threading.Thread):
    """Listens for UDP command heartbeats and packets from the GCS Laptop.

    Monitors link timeouts and notifies MasterTelecom when GCS disconnects.
    """

    def __init__(
        self, 
        port: int = 5007, 
        timeout_sec: float = 3.0,
        on_ip_discovered: Optional[Callable[[str], None]] = None,
        on_connection_lost: Optional[Callable[[], None]] = None,
        on_command_received: Optional[Callable[[str, Dict[str, Any], str], None]] = None
    ):
        super().__init__(daemon=True)
        self.port = port
        self.timeout_sec = timeout_sec
        self.on_ip_discovered = on_ip_discovered
        self.on_connection_lost = on_connection_lost
        self.on_command_received = on_command_received

        self.running = False
        self.active_gcs_ip: Optional[str] = None
        self.last_heartbeat: float = 0.0
        self.is_gcs_connected: bool = False
        self.sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def run(self):
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            self.sock.bind(("0.0.0.0", self.port))
        except Exception as e:
            print(f"[PC_BRIDGE_ERR] Failed to bind to port {self.port}: {e}")
            return

        self.sock.settimeout(0.5)
        print(f"[PC_BRIDGE] Listening for incoming GCS commands and heartbeats on 0.0.0.0:{self.port}...")

        while self.running:
            now = time.time()

            # Connection Timeout Evaluation
            with self._lock:
                if self.is_gcs_connected and (now - self.last_heartbeat > self.timeout_sec):
                    self.is_gcs_connected = False
                    print(f"[PC_BRIDGE] GCS Connection lost due to timeout (> {self.timeout_sec}s)")
                    if self.on_connection_lost:
                        self.on_connection_lost()

            try:
                data, addr = self.sock.recvfrom(1024)
                if data:
                    payload = json.loads(data.decode("utf-8"))
                    cmd = payload.get("cmd")
                    discovered_ip = addr[0]

                    with self._lock:
                        self.last_heartbeat = now
                        was_connected = self.is_gcs_connected
                        self.is_gcs_connected = True

                    if cmd == "GCS_CONNECT":
                        with self._lock:
                            ip_changed = self.active_gcs_ip != discovered_ip
                            self.active_gcs_ip = discovered_ip

                        if (ip_changed or not was_connected) and self.on_ip_discovered:
                            print(f"[PC_BRIDGE] Connection established from Laptop GCS IP: {discovered_ip}")
                            self.on_ip_discovered(discovered_ip)

                    # Forward command payload to MasterTelecom callback handler
                    if self.on_command_received and cmd:
                        self.on_command_received(cmd, payload, discovered_ip)

            except socket.timeout:
                pass
            except Exception as e:
                print(f"[PC_BRIDGE_ERR] Error processing packet: {e}")

        if self.sock:
            self.sock.close()

    def get_active_ip(self) -> Optional[str]:
        with self._lock:
            return self.active_gcs_ip

    def stop(self):
        self.running = False