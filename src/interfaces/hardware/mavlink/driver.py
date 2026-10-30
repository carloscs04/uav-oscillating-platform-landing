import time
from pymavlink import mavutil

class MAVLinkDriver:
    """Low-level pymavlink reader/writer."""
    
    def __init__(self, uri: str, baud_rate: int):
        self.uri = uri
        self.baud_rate = baud_rate
        self.master = None

    def open_connection(self) -> bool:
        if "udp:" in self.uri:
            self.master = mavutil.mavlink_connection(self.uri)
        else:
            self.master = mavutil.mavlink_connection(self.uri, baud=self.baud_rate)
        return self.master is not None

    def wait_for_heartbeat(self, timeout: float) -> dict | None:
        if not self.master:
            return None
        # Use wait_heartbeat instead of wait_for_heartbeat
        hb = self.master.wait_heartbeat(timeout=timeout)
        if hb:
            return {
                "system_id": self.master.target_system,
                "component_id": self.master.target_component,
            }
        return None

    def close(self):
        if self.master:
            self.master.close()
            self.master = None