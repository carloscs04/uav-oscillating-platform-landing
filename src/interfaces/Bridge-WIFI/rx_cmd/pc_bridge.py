# interfaces/Bridge-WIFI/rx_cmd/pc_bridge.py

import json
import socket
import threading
import time
from typing import Callable, Optional


class PCBridgeListener(threading.Thread):
    """Escucha el handshake UDP enviado por pi_bridge.py desde la laptop GCS.

    Captura automáticamente la dirección IP de la computadora y la reporta al
    master_telecom.
    """

    def __init__(self, port: int = 5007, on_ip_discovered: Optional[Callable[[str], None]] = None):
        super().__init__(daemon=True)
        self.port = port
        self.on_ip_discovered = on_ip_discovered

        self.running = False
        self.active_gcs_ip: Optional[str] = None
        self.sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def run(self):
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            self.sock.bind(("0.0.0.0", self.port))
        except Exception as e:
            print(f"[PC_BRIDGE_ERR] No se pudo vincular al puerto {self.port}: {e}")
            return

        self.sock.settimeout(0.5)
        print(f"[PC_BRIDGE] Escuchando handshake en 0.0.0.0:{self.port}...")

        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                if data:
                    payload = json.loads(data.decode("utf-8"))
                    cmd = payload.get("cmd")

                    if cmd == "GCS_CONNECT":
                        discovered_ip = addr[0]

                        with self._lock:
                            ip_changed = self.active_gcs_ip != discovered_ip
                            self.active_gcs_ip = discovered_ip

                        if ip_changed:
                            print(f"[PC_BRIDGE] Handshake recibido desde Laptop GCS IP: {discovered_ip}")
                            if self.on_ip_discovered:
                                self.on_ip_discovered(discovered_ip)

            except socket.timeout:
                pass
            except Exception as e:
                print(f"[PC_BRIDGE_ERR] Error procesando paquete: {e}")

        if self.sock:
            self.sock.close()

    def get_active_ip(self) -> Optional[str]:
        """Retorna la última IP registrada de forma thread-safe."""
        with self._lock:
            return self.active_gcs_ip

    def stop(self):
        self.running = False