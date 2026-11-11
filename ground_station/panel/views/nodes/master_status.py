# File: ground_station/panel/views/nodes/master_status_view.py

import tkinter as tk
from ground_station.utils.colors import (
    BG_CARD, FG_CYAN, FG_WHITE, ACCENT_GREEN, ACCENT_RED, ACCENT_YELLOW
)
from ground_station.comms.pi_bridge import PiLink, LinkState, HealthState
from ground_station.receivers.nodes.master_rx import MasterReceiver


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

        self.lbl_metrics = tk.Label(row, text="IP: N/A | Rate: -- Hz", font=("Consolas", 9), fg=FG_WHITE, bg=BG_CARD)
        self.lbl_metrics.pack(side="right")

        self._start_loop()

    def _start_loop(self):
        self.update_status()
        self.after(500, self._start_loop)

    def update_view(self):
        """Interfaz principal llamada por el loop GUI del MasterPanel."""
        self.update_status()

    def update_status(self):
        active_link = self.pi_link if hasattr(self.pi_link, "get_health") else getattr(PiLink, "_active_instance", None)
        master_rx = getattr(MasterReceiver, "_active_instance", None)

        is_connected = False
        ip_addr = "N/A"
        latency = "--"
        rate_hz = 0.0

        if active_link and hasattr(active_link, "get_health"):
            health = active_link.get_health()
            if health.get("state") == LinkState.CONNECTED:
                is_connected = True
                ip_addr = health.get("ip", "N/A")
                latency = health.get("latency_ms", "--")

        if master_rx and hasattr(master_rx, "get_health_info"):
            rx_health = master_rx.get_health_info()
            rx_status = rx_health.get("status", "OFFLINE")
            rate_hz = rx_health.get("rate_hz", 0.0)
            if rx_status in ["HEALTHY", "POOR"] or master_rx.last_seen > 0:
                is_connected = True
                if ip_addr == "N/A" and master_rx.last_rpi_ip:
                    ip_addr = master_rx.last_rpi_ip

        if is_connected:
            self.lbl_status.config(text="[MASTER ACTIVE / HEALTHY]", fg=ACCENT_GREEN)
            self.lbl_metrics.config(text=f"IP: {ip_addr} | Ping: {latency} ms | {rate_hz:.1f}Hz")
        else:
            self.lbl_status.config(text="[DISCONNECTED]", fg=ACCENT_RED)
            self.lbl_metrics.config(text="IP: N/A | Rate: -- Hz")