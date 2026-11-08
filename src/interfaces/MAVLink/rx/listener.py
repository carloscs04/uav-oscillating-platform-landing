import time
import threading
from typing import Any, Dict, Optional
from src.interfaces.MAVLink.state.vehicle_state import VehicleState


class MAVLinkRXListener:
    """Non-blocking high-frequency MAVLink listener loop updating VehicleState with watchdog metrics."""

    def __init__(self, state: VehicleState):
        self.state = state
        self.master: Optional[Any] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.last_recv_time: float = 0.0

    def set_master(self, master: Any):
        self.master = master
        self.last_recv_time = time.time()

    def wait_for_heartbeat(self, timeout: float = 5.0) -> Optional[Dict[str, Any]]:
        if not self.master:
            return None
        try:
            hb = self.master.wait_heartbeat(timeout=timeout)
            if hb:
                self.last_recv_time = time.time()
                return {"system_id": self.master.target_system}
        except Exception:
            pass
        return None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self.last_recv_time = time.time()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.2)
        self._thread = None
        self.master = None

    def _read_loop(self):
        while not self._stop_event.is_set() and self.master:
            try:
                msg = self.master.recv_match(blocking=False)
                if msg:
                    data = msg.to_dict()
                    pkt_type = data.get("mavpackettype")

                    if pkt_type and pkt_type != "BAD_DATA":
                        self.last_recv_time = time.time()
                        self.state.update_packet(pkt_type, data)
            except Exception:
                time.sleep(0.05)

            self._stop_event.wait(0.005)