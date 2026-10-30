import os
import time
import yaml
from typing import TypedDict
from .driver import MAVLinkDriver

class LaunchResult(TypedDict):
    success: bool
    status_code: str
    message: str
    driver: MAVLinkDriver | None

class MAVLinkLauncher:
    def __init__(self, config_path: str = None):
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "config.yaml")
        
        self.config_path = config_path
        self.config = self._load_config()

    def _load_config(self) -> dict:
        with open(self.config_path, "r") as f:
            return yaml.safe_load(f)["mavlink"]

    def start_connection(self) -> LaunchResult:
        mode = self.config.get("active_mode", "SIMULATION")
        target_info = self.config["targets"].get(mode)
        settings = self.config["settings"]

        if not target_info:
            return {
                "success": False,
                "status_code": "CONFIG_ERROR",
                "message": f"Target mode '{mode}' not defined in config.yaml",
                "driver": None
            }

        uri = target_info["uri"]
        baud = target_info["baud_rate"]
        retries = settings["max_connection_retries"]
        timeout = settings["heartbeat_timeout"]

        driver = MAVLinkDriver(uri=uri, baud_rate=baud)

        for attempt in range(1, retries + 1):
            if driver.open_connection():
                hb = driver.wait_for_heartbeat(timeout=timeout)
                if hb:
                    return {
                        "success": True,
                        "status_code": "CONNECTED",
                        "message": f"Successfully connected to PX4 in {mode} mode on {uri}. System ID: {hb['system_id']}",
                        "driver": driver
                    }
            time.sleep(1.0)

        driver.close()
        return {
            "success": False,
            "status_code": "TIMEOUT_ERROR",
            "message": f"Failed to connect on {uri} after {retries} attempts.",
            "driver": None
        }