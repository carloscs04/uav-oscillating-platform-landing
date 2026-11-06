# src/interfaces/wifi_telecom/tx_nodes/control_tx.py

import json
import socket
import time
import math


class ControlTransmitter:
    """Raspberry Pi Transmitter: Executes bi-directional handshake before streaming live control data."""

    def __init__(self, gcs_ip: str = "127.0.0.1", port: int = 5007):
        self.gcs_ip = gcs_ip
        self.port = port
        self.running = False
        self.is_synced = False

    def start(self):
        """Starts connection handshake sequence followed by active telemetry streaming."""
        self.running = True
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.5)

        print(f"[RPi CONTROL_TX] Transmitter active. Target GCS IP: {self.gcs_ip}:{self.port}")

        # ---------------------------------------------------------------------
        # PHASE 1: BROADCAST INITIAL BEACON & WAIT FOR ACK
        # ---------------------------------------------------------------------
        beacon_count = 0
        while self.running and not self.is_synced:
            try:
                beacon_count += 1
                beacon_packet = json.dumps({
                    "cmd": "INIT_BEACON",
                    "code": "SYNC_REQ_0x1",
                    "node": "3-Axis Control Node",
                    "attempt": beacon_count
                }).encode("utf-8")

                sock.sendto(beacon_packet, (self.gcs_ip, self.port))
                print(f"[HANDSHAKE STEP 1] RPi sending 'INIT_BEACON' attempt #{beacon_count} to GCS ({self.gcs_ip}:{self.port})...")

                # Listen for GCS ACK confirmation
                try:
                    data, addr = sock.recvfrom(1024)
                    resp = json.loads(data.decode("utf-8"))

                    if resp.get("cmd") == "ACK_START_STREAM":
                        self.is_synced = True
                        print(f"[HANDSHAKE STEP 2] RPi received 'ACK_START_STREAM' from GCS ({addr[0]}:{addr[1]})!")
                        print(f"                   Status: {resp.get('status')} | Stopping initial beacon.")
                except socket.timeout:
                    pass

            except Exception as e:
                print(f"[RPi CONTROL_TX_ERR] Beacon transmission error: {e}")

            time.sleep(0.5)

        # ---------------------------------------------------------------------
        # PHASE 2: STREAM LIVE TELEMETRY
        # ---------------------------------------------------------------------
        print(f"[HANDSHAKE COMPLETE] RPi starting high-frequency telemetry transmission (20Hz)...")
        t = 0.0

        while self.running and self.is_synced:
            try:
                pos_ref = [round(math.sin(t), 2), round(math.cos(t), 2), 1.5]
                pos_act = [round(math.sin(t) * 0.96, 2), round(math.cos(t) * 0.96, 2), round(1.48 + 0.03 * math.sin(t * 2), 2)]

                telemetry_packet = json.dumps({
                    "ref": pos_ref,
                    "act": pos_act,
                    "timestamp": time.time()
                }).encode("utf-8")

                sock.sendto(telemetry_packet, (self.gcs_ip, self.port))
                t += 0.1
                time.sleep(0.05)  # 20 Hz loop rate

            except Exception as e:
                print(f"[RPi CONTROL_TX_ERR] Stream error: {e}")
                time.sleep(0.1)

        sock.close()

    def stop(self):
        """Stops transmitter execution."""
        self.running = False


if __name__ == "__main__":
    # Standalone execution test simulation
    tx = ControlTransmitter(gcs_ip="127.0.0.1", port=5007)
    tx.start()