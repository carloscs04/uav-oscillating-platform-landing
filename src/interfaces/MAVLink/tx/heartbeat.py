import threading
from typing import Any, Optional
from pymavlink import mavutil


class MAVLinkHeartbeatEmitter:
    """Sends continuous standard 1Hz MAVLink heartbeats without blocking shutdown."""

    def __init__(self):
        self.master: Optional[Any] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def set_master(self, master: Any):
        self.master = master

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.1)
        self._thread = None

    def _heartbeat_loop(self):
        while not self._stop_event.is_set() and self.master:
            try:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
            except Exception:
                pass
            self._stop_event.wait(1.0)  # Interruptible 1-second sleep