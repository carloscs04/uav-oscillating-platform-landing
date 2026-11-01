import time
import threading
from .telemetry_hub import TelemetryHub
from .uplink import UplinkReceiver
from .downlink import DownlinkSender
from .video_stream import VideoStreamer


class TelecomManager:
    """
    Self-running Wi-Fi Telecom Manager.
    - Operates its own internal background loop on start().
    - Uplink receiver runs continuously to maintain ground connection.
    - Pulls data from telemetry/frame providers and streams automatically.
    """

    def __init__(self, command_callback=None, telemetry_provider=None, frame_provider=None):
        self.hub = TelemetryHub()
        self.command_callback = command_callback
        
        # Data provider callbacks (functions passed from MasterNode / CameraDriver)
        self.telemetry_provider = telemetry_provider
        self.frame_provider = frame_provider

        # Sub-modules
        self.uplink = UplinkReceiver(self.hub, command_callback=self.command_callback)
        self.downlink = DownlinkSender(self.hub)
        self.video = VideoStreamer(self.hub)

        self.is_running = False
        self._loop_thread = None
        self.rate_hz = self.hub.config.get("telemetry_rate_hz", 20)

    def start(self):
        """Launches continuous RX/TX background threads and starts the internal loop."""
        if self.is_running:
            return

        print("[TelecomManager] Starting Wi-Fi communication engine...")
        self.is_running = True

        # 1. Start continuous uplink listener (Always active for incoming triggers)
        self.uplink.start()

        # 2. Start downlink and video services
        self.downlink.start()
        self.video.start()

        # 3. Start internal telecom execution while-loop thread
        self._loop_thread = threading.Thread(target=self._telecom_loop, daemon=True)
        self._loop_thread.start()

        print("[TelecomManager] Wi-Fi stack running autonomously.")

    def _telecom_loop(self):
        """Internal background loop running continuously while active."""
        interval = 1.0 / self.rate_hz

        while self.is_running:
            # Fetch latest telemetry snapshot via provider if available
            if self.telemetry_provider:
                telemetry_data = self.telemetry_provider()
                if isinstance(telemetry_data, dict):
                    # Attach manager health status automatically
                    telemetry_data.update(self.get_health_status())
                    self.downlink.push_telemetry(telemetry_data)

            # Fetch latest video frame via provider if available
            if self.frame_provider:
                frame = self.frame_provider()
                if frame is not None:
                    self.video.push_frame(frame)

            time.sleep(interval)

    def get_health_status(self) -> dict:
        """Returns health diagnostics of all telecom sub-modules."""
        active_ip = self.hub.get_pc_ip()

        return {
            "comms_active": self.is_running,
            "ground_connected": active_ip is not None,
            "ground_ip": active_ip,
            "uplink_alive": self.uplink.is_running and (self.uplink._thread is not None and self.uplink._thread.is_alive()),
            "downlink_alive": self.downlink.is_running and (self.downlink._thread is not None and self.downlink._thread.is_alive()),
            "video_alive": self.video.server is not None,
        }

    def stop(self):
        """Stops the internal while-loop and shuts down all network services."""
        print("[TelecomManager] Terminating Wi-Fi services...")
        self.is_running = False

        if self._loop_thread and self._loop_thread.is_alive():
            self._loop_thread.join(timeout=1.0)

        self.uplink.stop()
        self.downlink.stop()
        self.video.stop()
        print("[TelecomManager] Wi-Fi services offline.")