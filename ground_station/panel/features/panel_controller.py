# ground_station/panel/features/panel_controller.py

from typing import Tuple, Callable, List
from ground_station.panel.features.link_manager import LinkManager
from ground_station.panel.features.panel_nodes import NodeManager
from ground_station.transmiters.main_command_tx import MainCommandTX

class PanelController:
    """Master logic orchestrator binding Data Listeners, Link Safety, and Telemetry Models."""
    
    def __init__(self, config_dir: str, initial_mode: str = None):
        self.link_mgr = LinkManager(config_dir, initial_mode)
        self.node_mgr = NodeManager()
        self.cmd_tx = MainCommandTX(self.link_mgr)
        
        self.is_armed: bool = False
        self.in_flight: bool = False
        self.current_stage: str = "STAGE 1 (IDLE / READY)"
        self.master_state: str = "DISCONNECTED"
        
        self.pos_ref: List[float] = [0.0, 0.0, 1.5]
        self.pos_act: List[float] = [0.0, 0.0, 0.0]
        
        self.log_callback: Callable[[str], None] = None

    def set_log_callback(self, callback: Callable[[str], None]):
        self.log_callback = callback

    def _emit_log(self, text: str):
        if self.log_callback:
            self.log_callback(text)

    def start(self):
        self.link_mgr.start_link()
        self._emit_log("[SYSTEM] Connecting to Raspberry Pi Master Node...")

    def stop(self):
        self.link_mgr.stop_link()
        self.node_mgr.reset_all()
        self._emit_log("[SYSTEM] Ground Station session terminated.")

    def toggle_auto_reconnect(self) -> Tuple[bool, str]:
        ok, msg = self.link_mgr.toggle_auto_reconnect(self.is_armed, self.in_flight)
        if not ok:
            self._emit_log(f"[SAFETY_WARN] {msg}")
        else:
            self._emit_log(f"[NET] {msg}")
        return ok, msg

    def select_profile(self, mode: str) -> Tuple[bool, str]:
        ok, msg = self.link_mgr.select_profile(mode)
        self._emit_log(f"[NET] {msg}")
        return ok, msg

    def connect_profile(self, mode: str) -> Tuple[bool, str]:
        ok, msg = self.link_mgr.select_profile(mode)
        self._emit_log(f"[NET] Manual connect initiated for profile '{mode}'...")
        return ok, msg

    def disconnect_wifi(self) -> Tuple[bool, str]:
        ok, msg = self.link_mgr.manual_disconnect(self.is_armed, self.in_flight)
        if not ok:
            self._emit_log(f"[SAFETY_WARN] {msg}")
        else:
            self.master_state = "DISCONNECTED"
            self.node_mgr.reset_all()
            self._emit_log(f"[NET] {msg}")
        return ok, msg

    def send_cli_command(self, cmd: str):
        self._emit_log(f"GCS> {cmd}")
        if self.cmd_tx.send_command(cmd) and self.master_state == "ACTIVE":
            self._emit_log(f"[MASTER_NODE_ACK] Command '{cmd}' executed.")
        else:
            self._emit_log(f"[MASTER_NODE_ERR] Command '{cmd}' dropped — Master Node not ACTIVE.")

    def update_telemetry_loop(self):
        if self.is_armed or self.in_flight:
            self.link_mgr.auto_reconnect = True

        health = self.link_mgr.get_health()
        if health.get("state") == "HEALTHY":
            self.link_mgr.is_connecting = False
            self.master_state = "ACTIVE"
            latency = health.get("latency_ms")
            status = "POOR" if (isinstance(latency, (int, float)) and latency > 250) else "OK"
            self.node_mgr.update_node_data("Bridge/WiFi Node", {"latency": latency}, status)
        else:
            self.master_state = "DISCONNECTED"
            self.node_mgr.check_timeouts(timeout_sec=2.0)
            
            if self.link_mgr.auto_reconnect and not self.link_mgr.is_connecting:
                self.link_mgr.start_link()