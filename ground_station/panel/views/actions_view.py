# File: ground_station/panel/views/actions_view.py

import time
import tkinter as tk
from ground_station.utils.colors import BG_CARD, FG_CYAN
from ground_station.receivers.nodes.master_rx import MasterReceiver
from ground_station.transmiters.main_command_tx import MainCommandTransmitter


class ActionsView(tk.LabelFrame):
    """Stateful action control buttons synchronized with live RPi MasterNode telemetry."""

    def __init__(self, parent, controller=None, **kwargs):
        super().__init__(
            parent, text=" Operations & Action Controls ", font=("Helvetica", 10, "bold"),
            fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=8, **kwargs
        )
        self.controller = controller
        self.cmd_tx = getattr(MainCommandTransmitter, "_active_instance", None) or MainCommandTransmitter()

        self._live_armed: bool = False
        self._live_fsm_state: str = "OFFLINE"

        btn_grid = tk.Frame(self, bg=BG_CARD)
        btn_grid.pack(anchor="w", padx=10)

        b_style = {"font": ("Helvetica", 9, "bold"), "bd": 0, "padx": 10, "pady": 6, "cursor": "hand2"}

        self.btn_arm = tk.Button(
            btn_grid, text="⚡ ARM DRONE", bg="#d84315", fg="white",
            command=self.cmd_toggle_arm, **b_style
        )
        self.btn_arm.grid(row=0, column=0, padx=(0, 5))

        self.btn_mission = tk.Button(
            btn_grid, text="▶ START MISSION", bg="#00897b", fg="white",
            command=self.cmd_toggle_mission, **b_style
        )
        self.btn_mission.grid(row=0, column=1, padx=5)

        self.btn_hover = tk.Button(
            btn_grid, text="🛸 HOVER", bg="#00695c", fg="white",
            command=self.cmd_hover, **b_style
        )
        self.btn_hover.grid(row=0, column=2, padx=5)

        self.btn_land = tk.Button(
            btn_grid, text="🛬 LAND DRONE", bg="#0288d1", fg="white",
            command=self.cmd_land_drone, **b_style
        )
        self.btn_land.grid(row=0, column=3, padx=5)

        self.btn_abort = tk.Button(
            btn_grid, text="⚠ ABORT", bg="#c62828", fg="white",
            command=self.cmd_abort, **b_style
        )
        self.btn_abort.grid(row=0, column=4, padx=(5, 0))

        self._sync_ui_loop()

    def _sync_ui_loop(self):
        """Polls MasterReceiver telemetry to update button labels and actions dynamically."""
        rx = getattr(MasterReceiver, "_active_instance", None)
        armed = False
        fsm_state = "OFFLINE"

        if rx:
            last_seen = getattr(rx, "last_seen", 0.0)
            if last_seen > 0.0 and (time.time() - last_seen) <= 2.5:
                packets = rx.get_all_packets() if hasattr(rx, "get_all_packets") else {}
                m_status = packets.get("master_status", {}) if isinstance(packets, dict) else {}
                if isinstance(m_status, dict) and m_status:
                    armed = bool(m_status.get("armed", False))
                    fsm_state = str(m_status.get("system_state", "READY")).upper()

        self._live_armed = armed
        self._live_fsm_state = fsm_state

        if self.controller:
            self.controller.is_armed = armed
            self.controller.in_flight = (fsm_state == "MISSION_ACTIVE")

        # 1. Dynamic ARM / DISARM button
        if armed:
            self.btn_arm.config(text="🔓 DISARM DRONE", bg="#455a64")
        else:
            self.btn_arm.config(text="⚡ ARM DRONE", bg="#d84315")

        # 2. Dynamic START MISSION / STOP MISSION / RESET button
        if fsm_state == "EMERGENCY_ABORT":
            self.btn_mission.config(text="↻ RESET SYSTEM", bg="#f57c00")
        elif fsm_state == "MISSION_ACTIVE":
            self.btn_mission.config(text="⏹ STOP MISSION", bg="#e65100")
        else:
            self.btn_mission.config(text="▶ START MISSION", bg="#00897b")

        self.after(200, self._sync_ui_loop)

    def _dispatch_to_master(self, cli_command: str, log_msg: str) -> bool:
        rx = getattr(MasterReceiver, "_active_instance", None)
        if rx and hasattr(rx, "send_terminal_command"):
            sent = rx.send_terminal_command(cli_command)
        else:
            sent = self.cmd_tx.send_command(cli_command)

        if self.controller and hasattr(self.controller, "_emit_log"):
            if sent:
                self.controller._emit_log(log_msg)
            else:
                self.controller._emit_log("[ERROR] RPi MasterNode OFFLINE. Command not sent.")
        return sent

    def cmd_toggle_arm(self):
        if self._live_armed:
            self._dispatch_to_master("disarm", "[TX] 'disarm' sent to MasterNode.")
        else:
            self._dispatch_to_master("arm", "[TX] 'arm' sent to MasterNode.")

    def cmd_toggle_mission(self):
        if self._live_fsm_state == "EMERGENCY_ABORT":
            self._dispatch_to_master("reset", "[TX] 'reset' sent to MasterNode.")
        elif self._live_fsm_state == "MISSION_ACTIVE":
            self._dispatch_to_master("mission stop", "[TX] 'mission stop' sent to MasterNode.")
        else:
            # Confirmation prompt ('y'/'n') is handled directly inside the MasterNode terminal
            self._dispatch_to_master("mission start", "[TX] 'mission start' sent -> Confirm (y/n) in console.")

    def cmd_hover(self):
        self._dispatch_to_master("hover", "[TX] 'hover' sent to ControlManager.")

    def cmd_land_drone(self):
        self._dispatch_to_master("mission land", "[TX] 'mission land' sent to MasterNode.")

    def cmd_abort(self):
        self._dispatch_to_master("abort", "[EMERGENCY] 'abort' sent to MasterNode!")