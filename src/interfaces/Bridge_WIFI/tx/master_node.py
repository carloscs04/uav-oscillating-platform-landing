# interfaces/Bridge-WIFI/tx_nodes/master_tx.py

import json
import socket
import time
from typing import Optional


class MasterTransmitter:
    """Sends terminal output lines and master node status logs to the GCS."""

    def __init__(self, target_ip: str = "127.0.0.1", port: int = 5006):
        self.target_ip = target_ip
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def update_target_ip(self, new_ip: str):
        self.target_ip = new_ip

    def send_terminal_output(self, line_text: str):
        """Pipes a single line of terminal output directly to GCS UI."""
        if not self.target_ip:
            return

        payload = json.dumps({
            "node": "System Master Node",
            "cmd": "SHELL_OUTPUT",
            "line": line_text,
            "timestamp": time.time()
        }).encode("utf-8")

        try:
            self._sock.sendto(payload, (self.target_ip, self.port))
        except Exception as e:
            print(f"[MASTER_TX_ERR] Failed to pipe line to {self.target_ip}: {e}")

    def close(self):
        if self._sock:
            self._sock.close()