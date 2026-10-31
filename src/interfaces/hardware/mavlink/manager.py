import os
import yaml
from typing import Optional
from .driver import MAVLinkDriver


class MAVLinkManager:
    """Lifecycle controller: provisions, starts, monitors, and stops the MAVLinkDriver."""

    def __init__(self, config_path: Optional[str] = None):
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "config.yaml")

        self.config_path = config_path
        self.config = self._load_config()
        self.driver: Optional[MAVLinkDriver] = None

    def _load_config(self) -> dict:
        with open(self.config_path, "r") as f:
            return yaml.safe_load(f)["mavlink"]

    def start(self, mode: str = "SIMULATION") -> bool:
        """Starts the driver module with the tools needed for the given mode."""
        mode = mode.upper()
        target_info = self.config["targets"].get(mode)

        if not target_info:
            print(f"[Manager Error] Mode '{mode}' not found in config.yaml")
            return False

        uri = target_info["uri"]
        baud = target_info.get("baud_rate", 921600)
        timeout = self.config.get("settings", {}).get("heartbeat_timeout", 5.0)

        # Provision driver module
        self.driver = MAVLinkDriver(uri=uri, baud_rate=baud)

        if self.driver.open_connection():
            hb = self.driver.wait_for_heartbeat(timeout=timeout)
            if hb:
                print(f"[Manager] MAVLink driver started successfully on {uri} ({mode} mode)")
                return True

        # If startup fails, stop and cleanup
        self.stop()
        return False

    def is_healthy(self) -> bool:
        """Checks if the driver worker is alive and connected."""
        return self.driver is not None and self.driver.master is not None

    def stop(self):
        """Cleanly halts the driver worker and closes hardware channels."""
        if self.driver:
            print("[Manager] Shutting down MAVLink driver...")
            self.driver.close()
            self.driver = None
            print("[Manager] Driver stopped cleanly.")