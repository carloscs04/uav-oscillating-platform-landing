# File: ground_station/receivers/main_receiver.py

import time
from typing import Any, Dict

from ground_station.receivers.nodes.master_rx import MasterReceiver
from ground_station.receivers.nodes.mavlink_rx import MAVLinkReceiver
from ground_station.receivers.nodes.distance_sensor_rx import DistanceSensorReceiver
from ground_station.receivers.nodes.camera_rx import CameraReceiver
from ground_station.receivers.nodes.vision_rx import VisionReceiver
from ground_station.receivers.nodes.control_rx import ControlReceiver


class MainReceiver:
    """Aggregates telemetry and stream receivers across all node sub-receivers."""

    def __init__(self):
        self.master_rx = MasterReceiver(port=5001)
        self.distance_rx = DistanceSensorReceiver(port=5002)
        self.camera_rx = CameraReceiver(port=5003)      # Dedicated video stream for CameraView
        self.control_rx = ControlReceiver(port=5004)
        self.mavlink_rx = MAVLinkReceiver(port=5005)
        self.vision_rx = VisionReceiver(port=5006)      # Dedicated ArUco telemetry for Node Matrix & Terminal

    def start_all(self):
        """Starts background thread listener loops for all node receivers."""
        self.master_rx.start()
        self.mavlink_rx.start()
        self.distance_rx.start()
        self.camera_rx.start()
        self.vision_rx.start()
        self.control_rx.start()

    def stop_all(self):
        """Gracefully shuts down all active receiver background threads."""
        self.master_rx.stop()
        self.mavlink_rx.stop()
        self.distance_rx.stop()
        self.camera_rx.stop()
        self.vision_rx.stop()
        self.control_rx.stop()

    def get_all_node_statuses(self) -> Dict[str, Dict[str, Any]]:
        """Evaluates heartbeat timeouts across sub-nodes (excluding Master Node)."""
        now = time.time()

        def check_stat(last_seen: float, timeout: float = 2.5) -> str:
            return "OK" if (now - last_seen < timeout and last_seen > 0) else "DISCONNECTED"

        mav_info = self.mavlink_rx.get_status_info() if hasattr(self.mavlink_rx, "get_status_info") else {}
        dist_info = self.distance_rx.get_status_info() if hasattr(self.distance_rx, "get_status_info") else {}
        vis_info = self.vision_rx.get_status_info() if hasattr(self.vision_rx, "get_status_info") else {}
        ctrl_info = self.control_rx.get_status_info() if hasattr(self.control_rx, "get_status_info") else {}

        return {
            "MAVLink Node": {
                "status": check_stat(mav_info.get("last_seen", getattr(self.mavlink_rx, "last_seen", 0.0))),
                "port": getattr(self.mavlink_rx, "port", 5005),
                "rate": f"{mav_info.get('rate', 10.0)}Hz",
                "telemetry": mav_info.get("telemetry", getattr(self.mavlink_rx, "packets", {})),
            },
            "Distance-Sensor Node": {
                "status": check_stat(dist_info.get("last_seen", getattr(self.distance_rx, "last_seen", 0.0))),
                "port": getattr(self.distance_rx, "port", 5002),
                "rate": f"{dist_info.get('rate', 15.0)}Hz",
                "telemetry": dist_info.get("telemetry", getattr(self.distance_rx, "packets", {})),
            },
            "Vision-ArUco Node": {
                "status": check_stat(vis_info.get("last_seen", getattr(self.vision_rx, "last_seen", 0.0))),
                "port": getattr(self.vision_rx, "port", 5006),
                "rate": f"{vis_info.get('rate', 30.0)}Hz",
                "telemetry": vis_info.get("telemetry", getattr(self.vision_rx, "packets", {})),
            },
            "Control Node": {
                "status": check_stat(ctrl_info.get("last_seen", getattr(self.control_rx, "last_seen", 0.0))),
                "port": getattr(self.control_rx, "port", 5004),
                "rate": f"{ctrl_info.get('rate', 20.0)}Hz",
                "telemetry": ctrl_info.get("telemetry", getattr(self.control_rx, "packets", {})),
            },
        }