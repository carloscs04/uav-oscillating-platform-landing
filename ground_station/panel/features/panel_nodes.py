# ground_station/panel/features/panel_nodes.py

import time
from typing import Dict, Any

class NodeManager:
    """Manages telemetry states, heartbeat timeouts, and metrics for all ground station sub-nodes."""
    
    def __init__(self):
        self._nodes: Dict[str, Dict[str, Any]] = {
            "PX4-MAVLink Node": {
                "status": "DISCONNECTED",
                "port": 5005,
                "rate": "0Hz",
                "last_seen": 0.0,
                "pkt_count": 0,
                "telemetry": {}
            },
            "Distance-Sensor Node": {
                "status": "DISCONNECTED",
                "port": 5008,
                "rate": "0Hz",
                "last_seen": 0.0,
                "pkt_count": 0,
                "telemetry": {}
            },
            "Aruco-Perception Node": {
                "status": "DISCONNECTED",
                "port": 8080,
                "rate": "0fps",
                "last_seen": 0.0,
                "pkt_count": 0,
                "telemetry": {}
            },
            "Bridge/WiFi Node": {
                "status": "DISCONNECTED",
                "port": 5006,
                "rate": "0Hz",
                "last_seen": 0.0,
                "pkt_count": 0,
                "telemetry": {}
            }
        }
        self._last_rate_calc = time.time()

    def update_node_data(self, node_name: str, payload: dict, status: str = "OK"):
        """Called by receivers when fresh data packets arrive."""
        if node_name in self._nodes:
            now = time.time()
            node = self._nodes[node_name]
            node["status"] = status
            node["last_seen"] = now
            node["pkt_count"] += 1
            node["telemetry"].update(payload)

    def check_timeouts(self, timeout_sec: float = 2.0):
        """Watchdog loop checking for missing heartbeat data streams."""
        now = time.time()
        
        # Calculate rates every 1.0 seconds
        calc_rates = (now - self._last_rate_calc) >= 1.0
        
        for name, node in self._nodes.items():
            if now - node["last_seen"] > timeout_sec:
                node["status"] = "DISCONNECTED"
                node["rate"] = "0Hz" if "Node" in name else "0fps"
                node["pkt_count"] = 0
            elif calc_rates:
                unit = "fps" if "Aruco" in name else "Hz"
                node["rate"] = f"{node['pkt_count']}{unit}"
                node["pkt_count"] = 0
                
        if calc_rates:
            self._last_rate_calc = now

    def reset_all(self):
        """Flushes states to DISCONNECTED during link drops or manual disconnects."""
        for node in self._nodes.values():
            node["status"] = "DISCONNECTED"
            node["rate"] = "0Hz"
            node["pkt_count"] = 0
            node["telemetry"].clear()

    def get_node_info(self, node_name: str) -> dict:
        return self._nodes.get(node_name, {})

    def get_all_states(self) -> Dict[str, Dict[str, Any]]:
        return self._nodes