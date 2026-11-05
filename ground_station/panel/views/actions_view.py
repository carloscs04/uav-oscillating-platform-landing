# ground_station/panel/views/actions_view.py

import tkinter as tk
from tkinter import messagebox
from ground_station.utils.colors import BG_CARD, FG_CYAN

class ActionsView(tk.LabelFrame):
    """Left action control buttons (ARM, START, LAND, ABORT)."""
    
    def __init__(self, parent, controller, **kwargs):
        super().__init__(parent, text=" Operations & Action Controls ", font=("Helvetica", 10, "bold"),
                         fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=8, **kwargs)
        self.controller = controller

        btn_grid = tk.Frame(self, bg=BG_CARD)
        btn_grid.pack(anchor="w", padx=10)

        b_style = {"font": ("Helvetica", 9, "bold"), "bd": 0, "padx": 14, "pady": 6, "cursor": "hand2"}

        tk.Button(btn_grid, text="⚡ ARM DRONE", bg="#d84315", fg="white", command=self.cmd_arm, **b_style).grid(row=0, column=0, padx=(0, 8))
        tk.Button(btn_grid, text="▶ START MISSION", bg="#00897b", fg="white", command=self.cmd_start_mission, **b_style).grid(row=0, column=1, padx=8)
        tk.Button(btn_grid, text="🛬 LAND DRONE", bg="#0288d1", fg="white", command=self.cmd_land_drone, **b_style).grid(row=0, column=2, padx=8)
        tk.Button(btn_grid, text="⚠ ABORT", bg="#c62828", fg="white", command=self.cmd_abort, **b_style).grid(row=0, column=3, padx=8)

    def cmd_arm(self):
        self.controller.is_armed = True
        self.controller._emit_log("[TX] ARM DRONE packet sent to Master Node. Drone ARMED.")

    def cmd_start_mission(self):
        if not self.controller.is_armed:
            messagebox.showwarning("Arming Guard", "Must ARM drone before starting mission!")
            return
        self.controller.in_flight = True
        self.controller.current_stage = "STAGE 3 (LAUNCH MISSION)"
        self.controller._emit_log("[TX] START MISSION packet sent.")

    def cmd_land_drone(self):
        self.controller.in_flight = False
        self.controller.is_armed = False
        self.controller.current_stage = "STAGE 5 (LANDED / DISARMED)"
        self.controller._emit_log("[TX] LAND DRONE packet sent.")

    def cmd_abort(self):
        self.controller.in_flight = False
        self.controller.is_armed = False
        self.controller.current_stage = "EMERGENCY ABORT"
        self.controller._emit_log("[EMERGENCY] ABORT MISSION SENT!")