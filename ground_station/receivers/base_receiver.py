# ground_station/receivers/base_receiver.py

import threading

class BaseReceiver(threading.Thread):
    """Base UDP stream receiver class."""
    
    def __init__(self, node_name: str, port: int, node_manager):
        super().__init__(daemon=True)
        self.node_name = node_name
        self.port = port
        self.node_mgr = node_manager
        self.running = False

    def stop(self):
        self.running = False

# ground_station/receivers/mavlink_rx.py
class MAVLinkReceiver(BaseReceiver):
    def run(self):
        self.running = True
        while self.running:
            # Listening logic on UDP port 5005...
            pass

# ground_station/receivers/distance_sensor_rx.py
class DistanceSensorReceiver(BaseReceiver):
    def run(self):
        self.running = True
        while self.running:
            # Listening logic on UDP port 5008...
            pass

# ground_station/receivers/wifi_bridge_rx.py
class WiFiBridgeReceiver(BaseReceiver):
    def run(self):
        self.running = True
        while self.running:
            # Listening logic on UDP port 5006...
            pass

# ground_station/receivers/camera_rx.py
class CameraReceiver(BaseReceiver):
    def run(self):
        self.running = True
        while self.running:
            # Listening MJPEG stream on port 8080...
            pass