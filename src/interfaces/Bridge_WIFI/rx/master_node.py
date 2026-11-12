# File: src/interfaces/Bridge_WIFI/rx/master_node.py

import json
import socket
import threading
import time
from typing import Any, Optional


class MasterCommandReceiver(threading.Thread):
    """Dedicated UDP receiver inside Bridge_WIFI/rx that forwards GCS CLI commands to MasterNode."""

    def __init__(self, host: str = "0.0.0.0", port: int = 5009, master_node: Optional[Any] = None):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.master_node = master_node
        self.running = False
        self._sock: Optional[socket.socket] = None

    def set_master_node(self, node: Any):
        self.master_node = node

    def run(self):
        self.running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(0.5)

        try:
            self._sock.bind((self.host, self.port))
            print(f"[MASTER_RX] Listening for remote GCS CLI commands on {self.host}:{self.port}...")
        except Exception as e:
            print(f"[MASTER_RX_ERR] Failed to bind port {self.port}: {e}")
            return

        while self.running:
            try:
                data, _addr = self._sock.recvfrom(4096)
                if not data:
                    continue

                raw_str = data.decode("utf-8").strip()
                if not raw_str:
                    continue

                cmd_text = ""
                if raw_str.startswith("{"):
                    msg = json.loads(raw_str)
                    payload = msg.get("payload") if isinstance(msg.get("payload"), dict) else {}
                    cmd_text = (
                        msg.get("text")
                        or msg.get("command")
                        or payload.get("text")
                        or payload.get("command")
                        or ""
                    )
                else:
                    cmd_text = raw_str

                if cmd_text and self.master_node and hasattr(self.master_node, "execute_cli_command"):
                    self.master_node.execute_cli_command(str(cmd_text))

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[MASTER_RX_ERR] Command parse error: {e}")
                    time.sleep(0.05)

        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

    def stop(self):
        self.running = False