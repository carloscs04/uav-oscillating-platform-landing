# ground_station/receivers/nodes/control_rx.py

import json
import socket
import threading
import time
from typing import List, Tuple, Optional, Dict, Any

from ground_station.comms.pi_bridge import PiLink, LinkState


class ControlReceiver(threading.Thread):
    """Receives 3-axis control telemetry.

    Registers its live instance to ControlReceiver._active_instance so GUI
    views can bind automatically.
    """

    _active_instance = None  # Global class reference to active receiver thread

    def __init__(self, host: str = "0.0.0.0", port: int = 5007):
        super().__init__(daemon=True)
        ControlReceiver._active_instance = self  # Register active instance globally

        self.host = host
        self.port = port
        self.running = False
        self.enabled = True  # Flag de activación individual controlado desde la GUI

        self._lock = threading.Lock()
        self.rx_state = "DISCONNECTED"
        self.pos_ref: List[float] = [0.0, 0.0, 1.5]
        self.pos_act: List[float] = [0.0, 0.0, 0.0]
        self.last_seen: float = 0.0

        self.rpi_address: Optional[Tuple[str, int]] = None
        self.last_rpi_ip: Optional[str] = None
        self._sock: Optional[socket.socket] = None

    def _is_pi_link_connected(self) -> bool:
        """Directly inspects PiLink._active_instance.

        Returns True ONLY when the active GUI link state is LinkState.CONNECTED
        and self.enabled is True.
        """
        if not self.enabled:
            return False

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
            # 1. GCS IS DISCONNECTED OR NODE IS DISABLED
            # -----------------------------------------------------------------
            if not is_connected:
                with self._lock:
                    target_state = "DISABLED" if not self.enabled else "DISCONNECTED"
                    if self.rx_state != target_state:
                        print(f"[GCS CONTROL_RX] Receiver entering state: {target_state}")
                        self.rx_state = target_state
                        self.pos_act = [0.0, 0.0, 0.0]

                try:
                    # Drenar sockets entrantes sin procesar paquetes
                    while True:
                        self._sock.settimeout(0.01)
                        self._sock.recvfrom(2048)
                except (socket.timeout, Exception):
                    self._sock.settimeout(0.5)

                time.sleep(0.2)
                continue

            # -----------------------------------------------------------------
            # 2. GCS IS CONNECTED & NODE IS ENABLED -> STREAM TELEMETRY
            # -----------------------------------------------------------------
            with self._lock:
                if self.rx_state in ["DISCONNECTED", "DISABLED"]:
                    print("[GCS CONTROL_RX] PiLink CONNECTED & Node ENABLED! Re-initiating stream...")
                    self.rx_state = "WAIT_BEACON"
                    self._send_start_signal()  # <-- Notifica automáticamente a la RPi para reanudar el envío

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
        with self._lock:
            if self.enabled and self.rx_state not in ["DISCONNECTED", "DISABLED"]:
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

    def _send_start_signal(self):
        """Envia la orden START_STREAM a la RPi al reconectar si el nodo está habilitado."""
        if self._sock:
            start_pkt = json.dumps({
                "cmd": "START_STREAM",
                "node": "3-Axis Control Node"
            }).encode("utf-8")
            
            # Obtener IP de la RPi de múltiples fuentes posibles
            active_link = getattr(PiLink, "_active_instance", None)
            target_ip = self.last_rpi_ip or (self.rpi_address[0] if self.rpi_address else None)
            if not target_ip and active_link:
                target_ip = getattr(active_link, "active_ip", None)

            if target_ip and target_ip not in ["N/A", "127.0.0.1", "localhost"]:
                try:
                    self._sock.sendto(start_pkt, (target_ip, self.port))
                    print(f"[GCS CONTROL_RX] Sent 'START_STREAM' reconnect signal to RPi at {target_ip}:{self.port}")
                except Exception as e:
                    print(f"[GCS CONTROL_RX_ERR] Error enviando START_STREAM a {target_ip}: {e}")

    def _send_stop_signal(self):
        """Envia la orden STOP_STREAM directamente al puerto 5007 de la Raspberry Pi."""
        if self._sock:
            stop_pkt = json.dumps({
                "cmd": "STOP_STREAM",
                "node": "3-Axis Control Node"
            }).encode("utf-8")
            
            target_ip = self.last_rpi_ip or (self.rpi_address[0] if self.rpi_address else None)
            
            if target_ip:
                try:
                    self._sock.sendto(stop_pkt, (target_ip, self.port))
                    print(f"[GCS CONTROL_RX] Sent 'STOP_STREAM' command to RPi at {target_ip}:{self.port}")
                except Exception as e:
                    print(f"[GCS CONTROL_RX_ERR] Error enviando STOP_STREAM a {target_ip}: {e}")

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