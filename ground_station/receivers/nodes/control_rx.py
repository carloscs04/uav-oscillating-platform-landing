# ground_station/receivers/nodes/control_rx.py

import json
import socket
import threading
import time
from typing import List, Tuple, Optional, Dict, Any

from ground_station.comms.pi_bridge import PiLink, LinkState


class ControlReceiver(threading.Thread):
    """Modular, plug-and-play UDP receiver for 3-Axis Control Telemetry.

    Decodes multi-packet payloads (trajectory, io_data, errors, custom packets)
    and classifies them into thread-safe structures for GUI and analytics.
    """

    _active_instance = None  # Global reference to active receiver thread instance

    def __init__(self, host: str = "0.0.0.0", port: int = 5007):
        super().__init__(daemon=True)
        ControlReceiver._active_instance = self

        self.host = host
        self.port = port
        self.running = False
        self.enabled = True  # Sub-node enable toggle flag

        self._lock = threading.Lock()
        self.rx_state = "DISCONNECTED"
        self.last_seen: float = 0.0

        # General packet repository: stores incoming packets by packet type name
        self.packets: Dict[str, Any] = {}

        # Specialized convenience properties for direct access
        self.pos_ref: List[float] = [0.0, 0.0, 1.5]
        self.pos_act: List[float] = [0.0, 0.0, 0.0]
        self.io_inputs: List[float] = []
        self.io_outputs: List[float] = []
        self.control_errors: List[float] = [0.0, 0.0, 0.0]

        self.rpi_address: Optional[Tuple[str, int]] = None
        self.last_rpi_ip: Optional[str] = None
        self._sock: Optional[socket.socket] = None

    def get_packet(self, pkt_type: str, default: Any = None) -> Any:
        """Thread-safely retrieves a specific classified packet payload by type."""
        with self._lock:
            return self.packets.get(pkt_type, default)

    def get_all_packets(self) -> Dict[str, Any]:
        """Returns a deep copy of all currently stored packets."""
        with self._lock:
            return dict(self.packets)

    def get_control_data(self) -> Tuple[List[float], List[float]]:
        """Legacy getter for plotting views (extracts position reference and actual vectors)."""
        with self._lock:
            if self.enabled and self.rx_state not in ["DISCONNECTED", "DISABLED"]:
                return list(self.pos_ref), list(self.pos_act)
            return list(self.pos_ref), [0.0, 0.0, 0.0]

    def get_status_info(self) -> Dict[str, Any]:
        """Returns node health status and the latest classified telemetry dictionary."""
        with self._lock:
            return {
                "status": self.rx_state,
                "port": self.port,
                "rate": 20,
                "telemetry": dict(self.packets),
                "last_seen": self.last_seen
            }

    def _is_pi_link_connected(self) -> bool:
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

            # Handle network disconnection or disabled node
            if not is_connected:
                with self._lock:
                    target_state = "DISABLED" if not self.enabled else "DISCONNECTED"
                    if self.rx_state != target_state:
                        print(f"[GCS CONTROL_RX] Receiver entering state: {target_state}")
                        self.rx_state = target_state
                        self.pos_act = [0.0, 0.0, 0.0]

                try:
                    while True:
                        self._sock.settimeout(0.01)
                        self._sock.recvfrom(2048)
                except (socket.timeout, Exception):
                    self._sock.settimeout(0.5)

                time.sleep(0.2)
                continue

            with self._lock:
                if self.rx_state in ["DISCONNECTED", "DISABLED"]:
                    print("[GCS CONTROL_RX] Link active & Node ENABLED! Re-initiating stream...")
                    self.rx_state = "WAIT_BEACON"
                    self._send_start_signal()

            try:
                data, addr = self._sock.recvfrom(2048)
                if not data:
                    continue

                self.rpi_address = addr
                self.last_rpi_ip = addr[0]
                payload = json.loads(data.decode("utf-8"))
                cmd = payload.get("cmd")

                # Handle handshake beacons
                if cmd == "INIT_BEACON":
                    ack_packet = json.dumps({
                        "cmd": "ACK_START_STREAM",
                        "status": "HANDSHAKE_CONFIRMED",
                        "gcs_timestamp": time.time()
                    }).encode("utf-8")

                    self._sock.sendto(ack_packet, addr)

                    with self._lock:
                        self.rx_state = "STREAMING"
                        self.last_seen = time.time()
                    continue

                # Process multi-packet telemetry payload
                self._process_telemetry_payload(payload)

            except socket.timeout:
                continue
            except Exception:
                if self.running:
                    time.sleep(0.05)

        self._close_socket()

    def _process_telemetry_payload(self, payload: Dict[str, Any]):
        """Decodes incoming payload, extracts sub-packets, and classifies data into target containers."""
        with self._lock:
            self.rx_state = "STREAMING"
            self.last_seen = time.time()

            incoming_packets = payload.get("packets", {})

            # Fallback for legacy single-packet format
            if not incoming_packets and ("ref" in payload or "act" in payload):
                incoming_packets["trajectory"] = {
                    "ref": payload.get("ref", self.pos_ref),
                    "act": payload.get("act", self.pos_act)
                }

            # Store and classify incoming packets dynamically
            for pkt_type, pkt_data in incoming_packets.items():
                self.packets[pkt_type] = pkt_data

                # Direct routing to dedicated attributes for standard packet types
                if pkt_type == "trajectory":
                    self.pos_ref = pkt_data.get("ref", self.pos_ref)
                    self.pos_act = pkt_data.get("act", self.pos_act)
                elif pkt_type == "io_data":
                    self.io_inputs = pkt_data.get("inputs", self.io_inputs)
                    self.io_outputs = pkt_data.get("outputs", self.io_outputs)
                elif pkt_type == "errors":
                    self.control_errors = pkt_data.get("err", self.control_errors)

    def _send_start_signal(self):
        if self._sock:
            start_pkt = json.dumps({
                "cmd": "START_STREAM",
                "node": "3-Axis Control Node"
            }).encode("utf-8")
            
            active_link = getattr(PiLink, "_active_instance", None)
            target_ip = self.last_rpi_ip or (self.rpi_address[0] if self.rpi_address else None)
            if not target_ip and active_link:
                target_ip = getattr(active_link, "active_ip", None)

            if target_ip and target_ip not in ["N/A", "127.0.0.1", "localhost"]:
                try:
                    self._sock.sendto(start_pkt, (target_ip, self.port))
                except Exception as e:
                    print(f"[GCS CONTROL_RX_ERR] Error sending START_STREAM: {e}")

    def _send_stop_signal(self):
        if self._sock:
            stop_pkt = json.dumps({
                "cmd": "STOP_STREAM",
                "node": "3-Axis Control Node"
            }).encode("utf-8")
            target_ip = self.last_rpi_ip or (self.rpi_address[0] if self.rpi_address else None)
            if target_ip:
                try:
                    self._sock.sendto(stop_pkt, (target_ip, self.port))
                except Exception as e:
                    print(f"[GCS CONTROL_RX_ERR] Error sending STOP_STREAM: {e}")

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