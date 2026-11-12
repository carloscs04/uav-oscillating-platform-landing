# File: ground_station/panel/views/actions_view.py

import tkinter as tk
from tkinter import messagebox
from ground_station.utils.colors import BG_CARD, FG_CYAN
from ground_station.receivers.nodes.master_rx import MasterReceiver
from ground_station.transmiters.main_command_tx import MainCommandTransmitter


class ActionsView(tk.LabelFrame):
    """Left action control buttons (ARM, START, LAND, ABORT) wired to RPi MasterNode."""

    def __init__(self, parent, controller=None, **kwargs):
        super().__init__(
            parent, text=" Operations & Action Controls ", font=("Helvetica", 10, "bold"),
            fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=8, **kwargs
        )
        self.controller = controller
        self.cmd_tx = getattr(MainCommandTransmitter, "_active_instance", None) or MainCommandTransmitter()

        btn_grid = tk.Frame(self, bg=BG_CARD)
        btn_grid.pack(anchor="w", padx=10)

        b_style = {"font": ("Helvetica", 9, "bold"), "bd": 0, "padx": 14, "pady": 6, "cursor": "hand2"}

        tk.Button(btn_grid, text="⚡ ARM DRONE", bg="#d84315", fg="white", command=self.cmd_arm, **b_style).grid(row=0, column=0, padx=(0, 8))
        tk.Button(btn_grid, text="▶ START MISSION", bg="#00897b", fg="white", command=self.cmd_start_mission, **b_style).grid(row=0, column=1, padx=8)
        tk.Button(btn_grid, text="🛬 LAND DRONE", bg="#0288d1", fg="white", command=self.cmd_land_drone, **b_style).grid(row=0, column=2, padx=8)
        tk.Button(btn_grid, text="⚠ ABORT", bg="#c62828", fg="white", command=self.cmd_abort, **b_style).grid(row=0, column=3, padx=8)

    def _is_vehicle_armed(self) -> bool:
        """Checks live armed state from MasterReceiver first, falling back to local controller."""
        rx = getattr(MasterReceiver, "_active_instance", None)
        if rx and hasattr(rx, "get_packet"):
            m_status = rx.get_packet("master_status", {})
            if isinstance(m_status, dict) and "armed" in m_status:
                return bool(m_status.get("armed", False))
        return bool(getattr(self.controller, "is_armed", False))

    def _dispatch_to_master(self, cli_command: str, fallback_log: str):
        rx = getattr(MasterReceiver, "_active_instance", None)
        sent = False
        if rx and hasattr(rx, "send_terminal_command"):
            sent = rx.send_terminal_command(cli_command)
        else:
            sent = self.cmd_tx.send_command(cli_command)

        if not sent:
            messagebox.showerror("Master Link Offline", "No hay conexión con la Raspberry Pi (MasterNode OFFLINE).")
            return False

        if self.controller and hasattr(self.controller, "_emit_log"):
            self.controller._emit_log(fallback_log)
        return True

    def cmd_arm(self):
        if self._dispatch_to_master("arm", "[TX] 'arm' sent to MasterNode -> MAVLink."):
            if self.controller:
                self.controller.is_armed = True

    def cmd_start_mission(self):
        if not self._is_vehicle_armed():
            messagebox.showwarning("Arming Guard", "Must ARM drone before starting mission!")
            return
        if self._dispatch_to_master("mission start", "[TX] 'mission start' sent to MasterNode."):
            if self.controller:
                self.controller.in_flight = True
                self.controller.current_stage = "STAGE 3 (LAUNCH MISSION)"

    def cmd_land_drone(self):
        if self._dispatch_to_master("mission land", "[TX] 'mission land' sent to MasterNode -> MAVLink."):
            if self.controller:
                self.controller.in_flight = False
                self.controller.current_stage = "STAGE 5 (LANDING)"

    def cmd_abort(self):
        if self._dispatch_to_master("abort", "[EMERGENCY] 'abort' sent to MasterNode -> MAVLink!"):
            if self.controller:
                self.controller.in_flight = False
                self.controller.is_armed = False
                self.controller.current_stage = "EMERGENCY ABORT"