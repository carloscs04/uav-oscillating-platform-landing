# ground_station/receivers/main_receiver.py

import time
from typing import Any, Dict

from ground_station.receivers.nodes.camera_rx import CameraReceiver
from ground_station.receivers.nodes.control_rx import ControlReceiver
from ground_station.receivers.nodes.distance_sensor_rx import DistanceSensorReceiver
from ground_station.receivers.nodes.mavlink_rx import MAVLinkReceiver


class MainReceiver:
    """Aggregates telemetry and stream receivers across all node sub-receivers."""

    def __init__(self):
        self.mavlink_rx = MAVLinkReceiver()
        self.distance_rx = DistanceSensorReceiver()
        self.camera_rx = CameraReceiver()
        self.control_rx = ControlReceiver()

    def start_all(self):
        """Starts background thread listener loops for all sub-node receivers."""
        self.mavlink_rx.start()
        self.distance_rx.start()
        self.camera_rx.start()
        self.control_rx.start()

    def stop_all(self):
        """Gracefully shuts down all active sub-node receiver background threads."""
        self.mavlink_rx.stop()
        self.distance_rx.stop()
        self.camera_rx.stop()
        self.control_rx.stop()

    def get_all_node_statuses(self) -> Dict[str, Dict[str, Any]]:
        """Evaluates heartbeat timeouts across nodes and compiles status matrix."""
        now = time.time()

        def check_stat(last_seen: float, timeout: float = 2.0) -> str:
            return "OK" if (now - last_seen < timeout and last_seen > 0) else "DISCONNECTED"

        ref, act = self.control_rx.get_control_data()

        return {
            "PX4-MAVLink Node": {
                "status": check_stat(getattr(self.mavlink_rx, "last_seen", 0.0)),
                "port": getattr(self.mavlink_rx, "port", 5005),
                "rate": "10Hz",
                "telemetry": getattr(self.mavlink_rx, "telemetry", {}),
            },
            "Distance-Sensor Node": {
                "status": check_stat(getattr(self.distance_rx, "last_seen", 0.0)),
                "port": getattr(self.distance_rx, "port", 5008),
                "rate": "5Hz",
                "telemetry": {"distance": getattr(self.distance_rx, "distance", 0.0)},
            },
            "Aruco-Perception Node": {
                "status": check_stat(getattr(self.camera_rx, "last_seen", 0.0)),
                "port": getattr(self.camera_rx, "port", 8080),
                "rate": "30fps",
                "telemetry": {},
            },
            "3-Axis Control Node": {
                "status": check_stat(getattr(self.control_rx, "last_seen", 0.0)),
                "port": getattr(self.control_rx, "port", 5007),
                "rate": "20Hz",
                "telemetry": {"ref": ref, "act": act},
            },
        }