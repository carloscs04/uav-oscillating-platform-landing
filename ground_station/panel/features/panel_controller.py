# ground_station/panel/features/panel_controller.py

import os
from typing import Dict, List, Tuple, Callable
from ground_station.comms.pi_bridge import PiLink
from ground_station.utils.yaml_loader import load_yaml

class PanelController:
    """Manages GCS telemetry state, network safety rules, and command dispatch."""
    
    def __init__(self, config_dir: str, initial_mode: str = None):
        self.config_dir = config_dir
        self.profiles_path = os.path.join(self.config_dir, "profiles.yaml")
        
        # Dynamic profile loading
        profiles_data = load_yaml(self.profiles_path)
        self.profiles: Dict[str, dict] = profiles_data.get("profiles", {})
        self.available_profiles: List[str] = list(self.profiles.keys())
        
        default_mode = initial_mode if (initial_mode and initial_mode in self.available_profiles) else (self.available_profiles[0] if self.available_profiles else "wifi")
        self.link = PiLink(self.config_dir, mode=default_mode)
        
        # Operational Flight States
        self.is_armed: bool = False
        self.in_flight: bool = False
        self.current_stage: str = "STAGE 1 (IDLE / READY)"
        
        # System Master Node State (Defaults to DISCONNECTED)
        self.master_state: str = "DISCONNECTED"
        
        # Auto-Reconnect Switch (Starts Enabled)
        self.auto_reconnect: bool = True
        self.is_connecting: bool = False
        
        # All Sub-Nodes Health States (Default strictly to DISCONNECTED)
        self.node_states = {
            "PX4-MAVLink Node": {"status": "DISCONNECTED", "port": 5005, "rate": "0Hz"},
            "Distance-Sensor Node": {"status": "DISCONNECTED", "port": 5008, "rate": "0Hz"},
            "Aruco-Perception Node": {"status": "DISCONNECTED", "port": 8080, "rate": "0fps"},
            "Bridge/WiFi Node": {"status": "DISCONNECTED", "port": 5006, "rate": "0Hz"}
        }
        
        self.pos_ref: List[float] = [0.0, 0.0, 1.5]
        self.pos_act: List[float] = [0.0, 0.0, 0.0]
        
        # Console Log Stream Callback
        self.log_callback: Callable[[str], None] = None

    def set_log_callback(self, callback: Callable[[str], None]):
        self.log_callback = callback

    def _emit_log(self, text: str):
        if self.log_callback:
            self.log_callback(text)

    def start(self):
        self.is_connecting = True
        self.link.start()
        self._emit_log("[SYSTEM] Connecting to Raspberry Pi Master Node...")

    def stop(self):
        self.is_connecting = False
        self.link.stop()
        self._emit_log("[SYSTEM] Ground Station session terminated.")

    def get_link_health(self) -> dict:
        health = self.link.get_health()
        if health.get("state") != "HEALTHY":
            health["latency_ms"] = "N/A"
        return health

    def is_link_connected(self) -> bool:
        health = self.get_link_health()
        return health.get("state") == "HEALTHY"

    def toggle_auto_reconnect(self) -> Tuple[bool, str]:
        """Toggles automatic connection retries with flight safety checks."""
        if (self.is_armed or self.in_flight) and self.auto_reconnect:
            msg = "SAFETY BLOCK: Auto-Reconnect MUST remain ON while drone is ARMED or IN-FLIGHT!"
            self._emit_log(f"[SAFETY_WARN] {msg}")
            return False, msg
            
        self.auto_reconnect = not self.auto_reconnect
        state_str = "ENABLED" if self.auto_reconnect else "DISABLED"
        
        if self.auto_reconnect:
            self._emit_log("[NET] Auto-Reconnect turned ON.")
            if not self.is_link_connected():
                self.is_connecting = True
                self.link.start()
                self._emit_log("[NET] Auto-Reconnect initiating connection...")
        else:
            self.is_connecting = False
            self.link.stop()
            self.master_state = "DISCONNECTED"
            for node in self.node_states.values():
                node["status"] = "DISCONNECTED"
            self._emit_log("[NET] Auto-Reconnect turned OFF. Connection attempt halted.")
            
        return True, f"Auto-Reconnect {state_str}"

    def can_disconnect_wifi(self) -> Tuple[bool, str]:
        if self.is_armed or self.in_flight:
            return False, "SAFETY BLOCK: Cannot disconnect Wi-Fi while drone is ARMED or IN-FLIGHT!"
        return True, "Safe to disconnect."

    def select_profile(self, mode: str) -> Tuple[bool, str]:
        """Handles profile dropdown change respecting the Auto-Reconnect toggle."""
        if mode not in self.available_profiles:
            msg = f"Invalid connection profile '{mode}'."
            self._emit_log(f"[NET_ERR] {msg}")
            return False, msg
        
        was_connected = self.is_link_connected() or self.is_connecting
        self.link.stop()
        self.link = PiLink(self.config_dir, mode=mode)

        if self.auto_reconnect or was_connected:
            self.is_connecting = True
            self.link.start()
            msg = f"Profile switched to '{mode}'. Initiating connection..."
        else:
            self.is_connecting = False
            self.master_state = "DISCONNECTED"
            for node in self.node_states.values():
                node["status"] = "DISCONNECTED"
            msg = f"Profile switched to '{mode}'."

        self._emit_log(f"[NET] {msg}")
        return True, msg

    def connect_profile(self, mode: str) -> Tuple[bool, str]:
        """Manual connection trigger via Connect button."""
        if mode not in self.available_profiles:
            msg = f"Invalid connection profile '{mode}'."
            self._emit_log(f"[NET_ERR] {msg}")
            return False, msg
        
        self.is_connecting = True
        self.link.stop()
        self.link = PiLink(self.config_dir, mode=mode)
        self.link.start()
        
        msg = f"Manual connection initiated for profile '{mode}'..."
        self._emit_log(f"[NET] {msg}")
        return True, msg

    def disconnect_wifi(self) -> Tuple[bool, str]:
        allowed, msg = self.can_disconnect_wifi()
        if not allowed:
            self._emit_log(f"[SAFETY_WARN] {msg}")
            return False, msg
        
        self.is_connecting = False
        self.auto_reconnect = False  # Explicitly turn Auto-Reconnect OFF on manual disconnect
        self.link.stop()
        self.master_state = "DISCONNECTED"
        for node in self.node_states.values():
            node["status"] = "DISCONNECTED"
        
        self._emit_log("[NET] Wi-Fi link disconnected by operator. Auto-Reconnect set to OFF.")
        return True, "Wi-Fi link disconnected safely."

    def send_cli_command(self, cmd: str):
        self._emit_log(f"GCS> {cmd}")
        if self.is_link_connected() and self.master_state == "ACTIVE":
            self._emit_log(f"[MASTER_NODE_ACK] Command '{cmd}' received & executing.")
        else:
            self._emit_log(f"[MASTER_NODE_ERR] Command '{cmd}' dropped — Master Node not ACTIVE.")

    def update_telemetry_from_pi(self):
        """Updates health status according to live Raspberry Pi telemetry link."""
        # Enforce Safety Rule: Auto-Reconnect must be ON during active flight
        if self.is_armed or self.in_flight:
            self.auto_reconnect = True

        health = self.get_link_health()
        link_state = health.get("state", "OFFLINE")

        if link_state == "HEALTHY":
            self.is_connecting = False
            latency = health.get("latency_ms")
            
            if isinstance(latency, (int, float)) and latency > 250:
                self.node_states["Bridge/WiFi Node"]["status"] = "POOR"
            else:
                self.node_states["Bridge/WiFi Node"]["status"] = "OK"
        else:
            self.master_state = "DISCONNECTED"
            for node in self.node_states.values():
                node["status"] = "DISCONNECTED"

            # Auto-Reconnect Trigger Logic (Only retries if auto_reconnect is ON)
            if self.auto_reconnect and not self.is_connecting and link_state in ["OFFLINE", "RECONNECTING", "DISCONNECTED"]:
                self.is_connecting = True
                self.link.start()