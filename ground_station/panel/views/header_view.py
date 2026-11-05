# ground_station/panel/views/header_view.py

import tkinter as tk
from tkinter import ttk
from ground_station.utils.colors import BG_CARD, FG_WHITE, FG_CYAN, FG_BLUE, ACCENT_GREEN, ACCENT_RED

class HeaderView(tk.Frame):
    """Header bar containing title, profile selector, connection button, and LED dot."""
    
    def __init__(self, parent, controller, **kwargs):
        super().__init__(parent, bg=BG_CARD, bd=1, relief="solid", pady=8, padx=15, **kwargs)
        self.controller = controller

        tk.Label(self, text="GROUND STATION MASTER PANEL", font=("Helvetica", 14, "bold"), fg=FG_WHITE, bg=BG_CARD).pack(side="left")

        conn_frame = tk.Frame(self, bg=BG_CARD)
        conn_frame.pack(side="right")

        tk.Label(conn_frame, text="Profile:", font=("Helvetica", 9, "bold"), fg=FG_CYAN, bg=BG_CARD).pack(side="left", padx=4)

        self.selected_profile = tk.StringVar(value=self.controller.link_mgr.link.active_mode)
        profile_dropdown = ttk.OptionMenu(
            conn_frame, self.selected_profile,
            self.controller.link_mgr.link.active_mode,
            *self.controller.link_mgr.available_profiles,
            command=self.controller.select_profile
        )
        profile_dropdown.pack(side="left", padx=4)

        self.conn_box = tk.Frame(conn_frame, bg=BG_CARD)
        self.conn_box.pack(side="left", padx=6)

        self.btn_toggle_conn = tk.Button(
            self.conn_box, text="🔌 Connect        ", bg="#00897b", fg="white",
            font=("Helvetica", 9, "bold"), padx=14, pady=4, bd=0, relief="flat", cursor="hand2",
            command=self.on_connect_click
        )
        self.btn_toggle_conn.pack(fill="both", expand=True)

        self.btn_auto_dot = tk.Label(
            self.btn_toggle_conn, text="●", font=("Helvetica", 8, "bold"),
            bg="#00897b", fg=ACCENT_GREEN, bd=0, relief="flat", highlightthickness=0, cursor="hand2"
        )
        self.btn_auto_dot.place(relx=1.0, rely=0.0, x=-4, y=2, anchor="ne")
        self.btn_auto_dot.bind("<Button-1>", lambda e: self.controller.toggle_auto_reconnect())

        self.status_bar = tk.Label(self, text="Initializing network link...", font=("Consolas", 10), fg=FG_BLUE, bg=BG_CARD)
        self.status_bar.pack(side="left", padx=15)

    def on_connect_click(self):
        if self.controller.link_mgr.is_connected():
            self.controller.disconnect_wifi()
        else:
            self.controller.connect_profile(self.selected_profile.get())

    def update_status(self, health: dict, is_connecting: bool, auto_reconnect: bool):
        link_state = health.get("state", "OFFLINE")
        lat = health.get("latency_ms")
        lat_str = f"{lat:.1f}ms" if isinstance(lat, (int, float)) and self.controller.link_mgr.is_connected() else "N/A ms"
        
        self.status_bar.config(
            text=f"Mode: [{health['mode'].upper()}] | IP: {health['ip']} | Link: [{link_state}] | Latency: {lat_str}"
        )

        if self.controller.link_mgr.is_connected():
            self.btn_toggle_conn.config(text="⏏ Disconnect        ", bg="#d32f2f")
        elif is_connecting or link_state in ["CONNECTING", "RECONNECTING"]:
            self.btn_toggle_conn.config(text="⏳ Connecting...     ", bg="#ffb300")
        else:
            self.btn_toggle_conn.config(text="🔌 Connect        ", bg="#00897b")

        btn_bg = self.btn_toggle_conn.cget("bg")
        dot_color = ACCENT_GREEN if auto_reconnect else ACCENT_RED
        self.btn_auto_dot.config(fg=dot_color, bg=btn_bg)