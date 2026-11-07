# ground_station/receivers/nodes/control_rx.py

import json
import socket
import threading
import time
from typing import List, Tuple, Optional, Dict, Any

from ground_station.comms.pi_bridge import PiLink, LinkState


class ControlReceiver(threading.Thread):
    """
    Receives 3-axis control telemetry.
    Registers its live instance to ControlReceiver._active_instance so GUI views can bind automatically.
    """

    _active_instance = None  # Global class reference to active receiver thread

    def __init__(self, host: str = "0.0.0.0", port: int = 5007):
        super().__init__(daemon=True)
        ControlReceiver._active_instance = self  # Register active instance globally

        self.host = host
        self.port = port
        self.running = False

        self._lock = threading.Lock()
        self.rx_state = "DISCONNECTED"
        self.pos_ref: List[float] = [0.0, 0.0, 1.5]
        self.pos_act: List[float] = [0.0, 0.0, 0.0]
        self.last_seen: float = 0.0

        self.rpi_address: Optional[Tuple[str, int]] = None
        self.last_rpi_ip: Optional[str] = None
        self._sock: Optional[socket.socket] = None

    def _is_pi_link_connected(self) -> bool:
        """
        Directly inspects PiLink._active_instance.
        Returns True ONLY when the active GUI link state is LinkState.CONNECTED.
        """
        active_link = getattr(PiLink, "_active_instance", None)
        if active_link is None:
            return False

        try:
            return active_link.link_state == LinkState.CONNECTED
        except Exception:
            return False

    def run(self):
        self.running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(0.5)

        try:
            self._sock.bind(("0.0.0.0", self.port))
            print(f"[GCS CONTROL_RX] Listener bound to 0.0.0.0:{self.port}")
        except Exception as e:
            print(f"[GCS CONTROL_RX_ERR] Bind error on port {self.port}: {e}")
            return

        while self.running:
            is_connected = self._is_pi_link_connected()

            # -----------------------------------------------------------------
            # 1. GCS IS DISCONNECTED
            # -----------------------------------------------------------------
            if not is_connected:
                with self._lock:
                    if self.rx_state != "DISCONNECTED":
                        print("[GCS CONTROL_RX] PiLink state != CONNECTED. Locking state & sending STOP signal.")
                        self._send_stop_signal()
                        self.rx_state = "DISCONNECTED"
                        self.pos_act = [0.0, 0.0, 0.0]

                try:
                    while True:
                        self._sock.settimeout(0.01)
                        self._sock.recvfrom(2048)
                except (socket.timeout, Exception):
                    self._sock.settimeout(0.5)

                time.sleep(0.2)
                continue

            # -----------------------------------------------------------------
            # 2. GCS IS CONNECTED -> ACCEPT BEACONS & STREAM
            # -----------------------------------------------------------------
            with self._lock:
                if self.rx_state == "DISCONNECTED":
                    print("[GCS CONTROL_RX] PiLink CONNECTED! Now accepting RPi handshakes...")
                    self.rx_state = "WAIT_BEACON"

            try:
                data, addr = self._sock.recvfrom(2048)
                if not data:
                    continue

                self.rpi_address = addr
                self.last_rpi_ip = addr[0]
                payload = json.loads(data.decode("utf-8"))
                cmd = payload.get("cmd")

                # Handshake Step 1: INIT_BEACON
                if cmd == "INIT_BEACON":
                    print(f"\n[HANDSHAKE STEP 1] Received 'INIT_BEACON' from RPi ({addr[0]}:{addr[1]})")

                    ack_packet = json.dumps({
                        "cmd": "ACK_START_STREAM",
                        "status": "HANDSHAKE_CONFIRMED",
                        "gcs_timestamp": time.time()
                    }).encode("utf-8")

                    self._sock.sendto(ack_packet, addr)
                    print(f"[HANDSHAKE STEP 2] Sent 'ACK_START_STREAM' reply to RPi at {addr[0]}:{addr[1]}")

                    with self._lock:
                        self.rx_state = "STREAMING"
                        self.last_seen = time.time()
                    continue

                # Live Telemetry Stream parsing
                if "ref" in payload or "act" in payload or "telemetry" in payload:
                    telemetry = payload.get("telemetry", payload)
                    with self._lock:
                        self.rx_state = "STREAMING"
                        self.pos_ref = telemetry.get("ref", self.pos_ref)
                        self.pos_act = telemetry.get("act", self.pos_act)
                        self.last_seen = time.time()

            except socket.timeout:
                continue
            except Exception:
                if self.running:
                    time.sleep(0.05)

        self._close_socket()

    def get_control_data(self) -> Tuple[List[float], List[float]]:
        """Returns live reference setpoint and actual position vectors whenever non-disconnected."""
        with self._lock:
            if self.rx_state != "DISCONNECTED":
                return list(self.pos_ref), list(self.pos_act)
            return list(self.pos_ref), [0.0, 0.0, 0.0]

    def get_status_info(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "status": self.rx_state,
                "port": self.port,
                "rate": 20,
                "telemetry": {
                    "ref": list(self.pos_ref),
                    "act": list(self.pos_act)
                },
                "last_seen": self.last_seen
            }

    def _send_stop_signal(self):
        if self._sock:
            stop_pkt = json.dumps({"cmd": "STOP_STREAM"}).encode("utf-8")
            target = self.rpi_address or ((self.last_rpi_ip, self.port) if self.last_rpi_ip else None)
            if target:
                try:
                    self._sock.sendto(stop_pkt, target)
                    print(f"[GCS CONTROL_RX] Sent 'STOP_STREAM' command to RPi at {target[0]}:{target[1]}")
                except Exception:
                    pass

    def _close_socket(self):
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    def stop(self):
        self.running = False
        self._send_stop_signal()
        with self._lock:
            self.rx_state = "DISCONNECTED"
        self._close_socket()