# File: ground_station/transmiters/main_command_tx.py

import json
import socket
import time
from typing import Optional
from ground_station.comms.pi_bridge import PiLink


class MainCommandTransmitter:
    """Outbound UDP command transmitter communicating directly with RPi MasterCommandReceiver (port 5009)."""

    _active_instance = None

    def __init__(self, link_manager=None, rpi_port: int = 5009):
        MainCommandTransmitter._active_instance = self
        self.link_mgr = link_manager
        self.rpi_port = rpi_port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _resolve_rpi_ip(self) -> Optional[str]:
        from ground_station.receivers.nodes.master_rx import MasterReceiver
        m_rx = getattr(MasterReceiver, "_active_instance", None)
        if m_rx and getattr(m_rx, "last_rpi_ip", None):
            return m_rx.last_rpi_ip

        active_link = self.link_mgr or getattr(PiLink, "_active_instance", None)
        if active_link:
            for attr in ("target_ip", "rpi_ip", "host", "ip"):
                ip_val = getattr(active_link, attr, None)
                if isinstance(ip_val, str) and ip_val:
                    return ip_val
        return None

    def send_command(self, cmd_text: str) -> bool:
        """Dispatches a single CLI command string to RPi Bridge_WIFI/rx/master_node.py (port 5009)."""
        clean_cmd = str(cmd_text).strip()
        if not clean_cmd:
            return False

        rpi_ip = self._resolve_rpi_ip()
        if rpi_ip:
            try:
                packet = json.dumps({
                    "cmd": "CLI_CMD",
                    "text": clean_cmd,
                    "timestamp": time.time()
                }).encode("utf-8")
                self._sock.sendto(packet, (rpi_ip, self.rpi_port))
                return True
            except Exception as e:
                print(f"[MAIN_CMD_TX_ERR] UDP send failed to {rpi_ip}:{self.rpi_port} -> {e}")

        # Fallback via PiLink only if dedicated UDP port 5009 target IP is unavailable
        active_link = self.link_mgr or getattr(PiLink, "_active_instance", None)
        if active_link and hasattr(active_link, "send_command"):
            active_link.send_command("CLI_CMD", {"text": clean_cmd})
            return True

        return False


MainReceiver = MainCommandTransmitter