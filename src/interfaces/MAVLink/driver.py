from pymavlink import mavutil


class MAVLinkDriver:
    """Low-level execution. Handles raw serial/UDP I/O."""
    
    def __init__(self, uri: str, baud_rate: int = 921600):
        self.uri = uri
        self.baud_rate = baud_rate
        self.master = None

    def open_connection(self) -> bool:
        if "udp:" in self.uri:
            self.master = mavutil.mavlink_connection(self.uri)
        else:
            self.master = mavutil.mavlink_connection(self.uri, baud=self.baud_rate)
        return self.master is not None

    def wait_for_heartbeat(self, timeout: float = 5.0) -> dict | None:
        if not self.master:
            return None
        hb = self.master.wait_heartbeat(timeout=timeout)
        return {"system_id": self.master.target_system} if hb else None

    def read_telemetry(self) -> dict | None:
        if not self.master:
            return None
        msg = self.master.recv_match(blocking=False)
        return msg.to_dict() if msg else None

    def send_velocity(self, vx: float, vy: float, vz: float):
        if not self.master:
            return
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0.0, 0.0, 0.0,
            vx, vy, vz,
            0.0, 0.0, 0.0,
            0.0, 0.0
        )

    def close(self):
        if self.master:
            self.master.close()
            self.master = None