# ground_station/panel/features/link_manager.py

import os
from typing import Dict, List, Tuple
from ground_station.comms.PiLink import PiLink
from ground_station.utils.yaml_loader import load_yaml

class LinkManager:
    """Manages low-level socket connections, dynamic profiles, and flight safety lockouts."""
    
    def __init__(self, config_dir: str, initial_mode: str = None):
        self.config_dir = config_dir
        self.profiles_path = os.path.join(self.config_dir, "profiles.yaml")
        
        profiles_data = load_yaml(self.profiles_path)
        self.profiles: Dict[str, dict] = profiles_data.get("profiles", {})
        self.available_profiles: List[str] = list(self.profiles.keys())
        
        default_mode = initial_mode if (initial_mode and initial_mode in self.available_profiles) else (self.available_profiles[0] if self.available_profiles else "wifi")
        self.link = PiLink(self.config_dir, mode=default_mode)
        
        self.auto_reconnect: bool = True
        self.is_connecting: bool = False

    def start_link(self):
        self.is_connecting = True
        self.link.start()

    def stop_link(self):
        self.is_connecting = False
        self.link.stop()

    def is_connected(self) -> bool:
        return self.get_health().get("state") == "HEALTHY"

    def get_health(self) -> dict:
        health = self.link.get_health()
        if health.get("state") != "HEALTHY":
            health["latency_ms"] = "N/A"
        return health

    def toggle_auto_reconnect(self, is_armed: bool, in_flight: bool) -> Tuple[bool, str]:
        if (is_armed or in_flight) and self.auto_reconnect:
            return False, "SAFETY BLOCK: Auto-Reconnect MUST remain ON while drone is ARMED or IN-FLIGHT!"
            
        self.auto_reconnect = not self.auto_reconnect
        if self.auto_reconnect:
            if not self.is_connected():
                self.start_link()
            return True, "Auto-Reconnect ENABLED"
        else:
            self.stop_link()
            return True, "Auto-Reconnect DISABLED"

    def select_profile(self, mode: str) -> Tuple[bool, str]:
        if mode not in self.available_profiles:
            return False, f"Invalid profile '{mode}'."
        
        was_connected = self.is_connected() or self.is_connecting
        self.stop_link()
        self.link = PiLink(self.config_dir, mode=mode)

        if self.auto_reconnect or was_connected:
            self.start_link()
            return True, f"Profile switched to '{mode}'. Connecting..."
        
        return True, f"Profile switched to '{mode}'."

    def manual_disconnect(self, is_armed: bool, in_flight: bool) -> Tuple[bool, str]:
        if is_armed or in_flight:
            return False, "SAFETY BLOCK: Cannot disconnect Wi-Fi while drone is ARMED or IN-FLIGHT!"
        
        self.auto_reconnect = False  # Turn Auto-Reconnect OFF on manual disconnect
        self.stop_link()
        return True, "Wi-Fi link disconnected safely."