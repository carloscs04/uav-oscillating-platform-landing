# ground_station/panel/core/panel_logic.py

from typing import Tuple, Callable, Any, List


class LogicOperation:
    """Central logic gatekeeper: tracks master safety state and validates permission requests."""

    def __init__(self):
        self.is_armed: bool = False
        self.in_flight: bool = False
        self.current_stage: str = "STAGE 1 (IDLE / READY)"
        self.master_state: str = "DISCONNECTED"

        self.pos_ref: List[float] = [0.0, 0.0, 1.5]
        self.pos_act: List[float] = [0.0, 0.0, 0.0]

        self.log_callback: Callable[[str], None] = None

    def status_features(self) -> dict:
        """Returns global state snapshot for view queries."""
        return {
            "is_armed": self.is_armed,
            "in_flight": self.in_flight,
            "current_stage": self.current_stage,
            "master_state": self.master_state,
        }

    def logic(self, action_type: str, payload: Any = None) -> Tuple[bool, str]:
        """Validates view permission requests against flight safety rules."""
        if action_type == "cmd":
            if not payload:
                return False, "Command string is empty."
            critical_cmds = ["reboot", "shutdown", "reset_params"]
            if self.in_flight and any(c in str(payload).lower() for c in critical_cmds):
                return False, f"SAFETY BLOCK: Command '{payload}' rejected while DRONE IS IN FLIGHT!"
            return True, "Command validated."

        if action_type in ["disconnect", "select_profile"]:
            if self.is_armed or self.in_flight:
                return False, f"SAFETY BLOCK: Cannot perform '{action_type}' while ARMED or IN-FLIGHT!"
            return True, "Action permitted."

        if action_type == "toggle_auto_reconnect":
            # payload is current auto_reconnect state
            if (self.is_armed or self.in_flight) and payload is True:
                return False, "SAFETY BLOCK: Auto-Reconnect MUST remain ON while ARMED or IN-FLIGHT!"
            return True, "Toggle permitted."

        return True, "Permitted."

    def set_log_callback(self, callback: Callable[[str], None]):
        self.log_callback = callback

    def emit_log(self, text: str):
        if self.log_callback:
            self.log_callback(text)