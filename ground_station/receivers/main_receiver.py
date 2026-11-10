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
        """Evaluates heartbeat timeouts across nodes and compiles full status matrix."""
        now = time.time()

        def check_stat(last_seen: float, timeout: float = 2.0) -> str:
            return "OK" if (now - last_seen < timeout and last_seen > 0) else "DISCONNECTED"

        # Safely query live status info dictionaries from each sub-receiver
        mav_info = self.mavlink_rx.get_status_info() if hasattr(self.mavlink_rx, "get_status_info") else {}
        dist_info = self.distance_rx.get_status_info() if hasattr(self.distance_rx, "get_status_info") else {}
        cam_info = self.camera_rx.get_status_info() if hasattr(self.camera_rx, "get_status_info") else {}
        ctrl_info = self.control_rx.get_status_info() if hasattr(self.control_rx, "get_status_info") else {}

        return {
            "MAVLink Node": {
                "status": check_stat(mav_info.get("last_seen", getattr(self.mavlink_rx, "last_seen", 0.0))),
                "port": getattr(self.mavlink_rx, "port", 5005),
                "rate": "10Hz",
                "telemetry": mav_info.get("telemetry", getattr(self.mavlink_rx, "packets", {})),
            },
            "Distance-Sensor Node": {
                "status": check_stat(dist_info.get("last_seen", getattr(self.distance_rx, "last_seen", 0.0))),
                "port": getattr(self.distance_rx, "port", 5008),
                "rate": "5Hz",
                "telemetry": dist_info.get("telemetry", getattr(self.distance_rx, "packets", {})),
            },
            "Camera-Aruco Node": {
                "status": check_stat(cam_info.get("last_seen", getattr(self.camera_rx, "last_seen", 0.0))),
                "port": getattr(self.camera_rx, "port", 8080),
                "rate": "30fps",
                "telemetry": cam_info.get("telemetry", getattr(self.camera_rx, "packets", {})),
            },
            "Control Node": {
                "status": check_stat(ctrl_info.get("last_seen", getattr(self.control_rx, "last_seen", 0.0))),
                "port": getattr(self.control_rx, "port", 5007),
                "rate": "20Hz",
                "telemetry": ctrl_info.get("telemetry", getattr(self.control_rx, "packets", {})),
            },
        }