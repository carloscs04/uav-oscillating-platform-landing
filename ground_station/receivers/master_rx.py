# ground_station/receivers/nodes/master_rx.py

import json
import socket
import threading
import time
from typing import Callable, Optional, Dict, Any

from ground_station.comms.pi_bridge import PiLink


class MasterTerminalBridge(threading.Thread):
    """GCS side interface to execute remote shell commands and stream output to UI."""

    _active_instance = None

    def __init__(self, port: int = 5006, on_terminal_line: Optional[Callable[[str], None]] = None):
        super().__init__(daemon=True)
        MasterTerminalBridge._active_instance = self
        self.port = port
        self.on_terminal_line = on_terminal_line

        self.running = False
        self._sock: Optional[socket.socket] = None

    def execute_remote_command(self, command_str: str) -> bool:
        """Sends a shell command string to be executed on the Raspberry Pi."""
        active_link = getattr(PiLink, "_active_instance", None)
        target_ip = getattr(active_link, "active_ip", None) if active_link else None

        if not target_ip or target_ip in ["N/A", "127.0.0.1", "localhost"]:
            print("[MASTER_BRIDGE_ERR] No valid RPi IP available to send command.")
            return False

        payload = json.dumps({
            "cmd": "EXEC_SHELL",
            "command": command_str,
            "timestamp": time.time()
        }).encode("utf-8")

        try:
            send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            send_sock.sendto(payload, (target_ip, self.port))
            send_sock.close()
            print(f"[MASTER_BRIDGE] Command sent to RPi ({target_ip}:{self.port}): '{command_str}'")
            return True
        except Exception as e:
            print(f"[MASTER_BRIDGE_ERR] Error sending shell command: {e}")
            return False

    def run(self):
        self.running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(0.5)

        try:
            self._sock.bind(("0.0.0.0", self.port))
            print(f"[GCS MASTER_BRIDGE] Output listener bound to 0.0.0.0:{self.port}")
        except Exception as e:
            print(f"[GCS MASTER_BRIDGE_ERR] Bind error on port {self.port}: {e}")
            return

        while self.running:
            try:
                data, addr = self._sock.recvfrom(4096)
                if not data:
                    continue

                payload = json.loads(data.decode("utf-8"))
                if payload.get("cmd") == "SHELL_OUTPUT":
                    line = payload.get("line", "")
                    if line and self.on_terminal_line:
                        self.on_terminal_line(line)

            except socket.timeout:
                continue
            except Exception:
                if self.running:
                    time.sleep(0.05)

        if self._sock:
            self._sock.close()

    def stop(self):
        self.running = False