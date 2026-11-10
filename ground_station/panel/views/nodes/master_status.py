# ground_station/panel/views/nodes/master_status_view.py

import tkinter as tk
from ground_station.utils.colors import (
    BG_CARD, FG_CYAN, FG_WHITE, ACCENT_GREEN, ACCENT_RED, ACCENT_YELLOW
)
from ground_station.comms.pi_bridge import PiLink, LinkState, HealthState


class MasterNodeView(tk.LabelFrame):
    """Global Master Telecom Connection & Health Status Widget."""

    def __init__(self, parent, pi_link=None, **kwargs):
        super().__init__(
            parent, text=" Master Node Telecom ", font=("Helvetica", 10, "bold"),
            fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=4, padx=8, **kwargs
        )
        self.pi_link = pi_link

        row = tk.Frame(self, bg=BG_CARD)
        row.pack(fill="x")

        tk.Label(row, text="MASTER LINK:", font=("Consolas", 9, "bold"), fg=FG_WHITE, bg=BG_CARD).pack(side="left")
        self.lbl_status = tk.Label(row, text="[OFFLINE]", font=("Consolas", 9, "bold"), fg=ACCENT_RED, bg=BG_CARD)
        self.lbl_status.pack(side="left", padx=6)

        self.lbl_metrics = tk.Label(row, text="IP: N/A | Ping: -- ms", font=("Consolas", 9), fg=FG_WHITE, bg=BG_CARD)
        self.lbl_metrics.pack(side="right")

        self._start_loop()

    def _start_loop(self):
        self.update_status()
        self.after(500, self._start_loop)

    def update_view(self):
        """Primary update interface called by MasterPanel GUI loop."""
        self.update_status()

    def update_status(self):
        active_link = self.pi_link if hasattr(self.pi_link, "get_health") else getattr(PiLink, "_active_instance", None)
        
        if not active_link or not hasattr(active_link, "get_health"):
            self.lbl_status.config(text="[OFFLINE]", fg=ACCENT_RED)
            self.lbl_metrics.config(text="IP: N/A | Ping: -- ms")
            return

        health = active_link.get_health()
        st = health.get("state", LinkState.DISCONNECTED)
        quality = health.get("health", HealthState.OFFLINE)
        ip = health.get("ip", "N/A")
        latency = health.get("latency_ms", "N/A")

        if st == LinkState.CONNECTED and quality == HealthState.HEALTHY:
            self.lbl_status.config(text="[MASTER ACTIVE / HEALTHY]", fg=ACCENT_GREEN)
        elif st == LinkState.CONNECTED:
            self.lbl_status.config(text="[DEGRADED]", fg=ACCENT_YELLOW)
        else:
            self.lbl_status.config(text="[DISCONNECTED]", fg=ACCENT_RED)

        self.lbl_metrics.config(text=f"IP: {ip} | Ping: {latency} ms")