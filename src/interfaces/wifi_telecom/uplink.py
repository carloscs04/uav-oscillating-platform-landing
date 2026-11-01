import socket
import threading
import time
from .telemetry_hub import TelemetryHub


class UplinkReceiver:
    """Self-contained UDP command receiver."""

    def __init__(self, hub: TelemetryHub, command_callback=None):
        self.hub = hub
        self.port = hub.config["telemetry_port"]
        self.command_callback = command_callback
        self.is_running = False

        self._sock = None
        self._thread = None

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(0.5)
        self._sock.bind(("0.0.0.0", self.port))

        self.is_running = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()
        print(f"[UplinkReceiver] Active on UDP port {self.port}")

    def _rx_loop(self):
        while self.is_running:
            try:
                data, addr = self._sock.recvfrom(1024)
                if data:
                    # Register sender IP into TelemetryHub
                    self.hub.register_pc_ip(addr[0])

                    command = data.decode("utf-8").strip().upper()
                    print(f"[Uplink] Trigger Received: '{command}'")

                    if self.command_callback:
                        self.command_callback(command)

            except socket.timeout:
                pass
            except Exception:
                if self.is_running:
                    time.sleep(0.2)

    def stop(self):
        self.is_running = False
        if self._sock:
            self._sock.close()