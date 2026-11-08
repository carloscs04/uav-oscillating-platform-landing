import os
import tkinter as tk
from tkinter import ttk
from ground_station.utils.colors import BG_CARD, FG_WHITE, FG_CYAN, FG_BLUE, ACCENT_GREEN, ACCENT_RED, ACCENT_YELLOW
from ground_station.utils.yaml_loader import load_yaml
from ground_station.comms.pi_bridge import PiLink, LinkState, HealthState


class HeaderView(tk.Frame):
    """Header View: Manages PiLink locally and displays connection state vs. health metrics."""

    def __init__(self, parent, logic_panel, config_dir: str, initial_mode: str = "school", **kwargs):
        super().__init__(parent, bg=BG_CARD, bd=1, relief="solid", pady=8, padx=15, **kwargs)
        self.logic_panel = logic_panel
        self.config_dir = config_dir

        self.link = PiLink(self.config_dir, mode=initial_mode)
        self.auto_connect: bool = True
        self.is_connecting: bool = False

        profiles_path = os.path.join(self.config_dir, "profiles.yaml")
        profiles_data = load_yaml(profiles_path)
        profiles_dict = profiles_data.get("profiles", {})
        self.available_profiles = list(profiles_dict.keys()) if profiles_dict else ["wifi", "usb", "ap"]

        tk.Label(self, text="GROUND STATION MASTER PANEL", font=("Helvetica", 14, "bold"), fg=FG_WHITE, bg=BG_CARD).pack(side="left")

        conn_frame = tk.Frame(self, bg=BG_CARD)
        conn_frame.pack(side="right")

        tk.Label(conn_frame, text="Profile:", font=("Helvetica", 9, "bold"), fg=FG_CYAN, bg=BG_CARD).pack(side="left", padx=4)

        self.selected_profile = tk.StringVar(value=self.link.active_mode)

        profile_dropdown = ttk.OptionMenu(
            conn_frame,
            self.selected_profile,
            self.link.active_mode,
            *self.available_profiles,
            command=self.on_profile_dropdown_change
        )
        profile_dropdown.pack(side="left", padx=4)

        self.conn_box = tk.Frame(conn_frame, bg=BG_CARD)
        self.conn_box.pack(side="left", padx=6)

        self.btn_toggle_conn = tk.Button(
            self.conn_box, text="🔌 Connect        ", bg="#00897b", fg="white",
            font=("Helvetica", 9, "bold"), padx=14, pady=4, bd=0, relief="flat", cursor="hand2",
            command=self.on_connect_button_click
        )
        self.btn_toggle_conn.pack(fill="both", expand=True)

        self.btn_auto_dot = tk.Label(
            self.btn_toggle_conn, text="●", font=("Helvetica", 8, "bold"),
            bg="#00897b", fg=ACCENT_GREEN, bd=0, relief="flat", highlightthickness=0, cursor="hand2"
        )
        self.btn_auto_dot.place(relx=1.0, rely=0.0, x=-4, y=2, anchor="ne")
        self.btn_auto_dot.bind("<Button-1>", lambda e: self.on_auto_connect_click())

        self.status_bar = tk.Label(self, text="Initializing network link...", font=("Consolas", 10), fg=FG_BLUE, bg=BG_CARD)
        self.status_bar.pack(side="left", padx=15)

    def on_connect_button_click(self):
        is_connected = self.link.get_health().get("state") == LinkState.CONNECTED

        if self.auto_connect:
            self.logic_panel.emit_log(f"[SAFETY_WARN] Auto-Connect is active. Toggle dot to disable.")
            return

        if is_connected or self.is_connecting:
            approved, msg = self.logic_panel.logic("disconnect")
            if not approved:
                self.logic_panel.emit_log(f"[SAFETY_WARN] {msg}")
                return
            
            self.is_connecting = False
            self.link.stop()
            self.logic_panel.master_state = "DISCONNECTED"
            self.logic_panel.emit_log("[NET] Wi-Fi link disconnected safely.")
        else:
            self.is_connecting = True
            self.link.start()
            self.logic_panel.emit_log(f"[NET] Connecting via profile '{self.selected_profile.get()}'...")

    def on_auto_connect_click(self):
        approved, msg = self.logic_panel.logic("toggle_auto_connect", payload=self.auto_connect)
        if not approved:
            self.logic_panel.emit_log(f"[SAFETY_WARN] {msg}")
            return

        self.auto_connect = not self.auto_connect
        if self.auto_connect:
            self.logic_panel.emit_log("[NET] Auto-Connect ENABLED.")
            if self.link.get_health().get("state") != LinkState.CONNECTED and not self.is_connecting:
                self.is_connecting = True
                self.link.start()
        else:
            self.is_connecting = False
            self.link.stop()
            self.logic_panel.emit_log("[NET] Auto-Connect DISABLED.")

    def on_profile_dropdown_change(self, new_mode: str):
        approved, msg = self.logic_panel.logic("select_profile", payload=new_mode)
        if not approved:
            self.logic_panel.emit_log(f"[SAFETY_WARN] {msg}")
            self.selected_profile.set(self.link.active_mode)
            return

        # Pre-lock connecting state to avoid UI refresh flicker during profile transition
        if self.auto_connect:
            self.is_connecting = True

        self.link.stop()
        self.link.load_profile(new_mode)

        if self.auto_connect:
            self.link.start()
            self.logic_panel.emit_log(f"[NET] Profile switched to '{new_mode}'. Auto-connecting...")
        else:
            self.is_connecting = False
            self.logic_panel.emit_log(f"[NET] Profile switched to '{new_mode}'. Connection stopped.")

    def update_header_ui(self):
        health = self.link.get_health()
        conn_state = health.get("state", LinkState.DISCONNECTED)
        link_health = health.get("health", HealthState.OFFLINE)

        if self.logic_panel.is_armed or self.logic_panel.in_flight:
            self.auto_connect = True

        # Gating loop to prevent socket collisions and UI flicker during transitional states
        if conn_state == LinkState.CONNECTED:
            self.is_connecting = False
            self.logic_panel.master_state = "ACTIVE"
        elif conn_state in [LinkState.CONNECTING, LinkState.RECONNECTING] or self.is_connecting:
            self.logic_panel.master_state = "DISCONNECTED"
        else:
            self.logic_panel.master_state = "DISCONNECTED"
            if self.auto_connect and not self.is_connecting:
                self.is_connecting = True
                self.link.start()

        lat = health.get("latency_ms")
        lat_str = f"{lat:.1f}ms" if isinstance(lat, (int, float)) and conn_state == LinkState.CONNECTED else "N/A ms"

        self.status_bar.config(
            text=f"Mode: [{health['mode'].upper()}] | IP: {health['ip']} | State: [{conn_state}] | Health: [{link_health}] ({lat_str})"
        )

        if conn_state == LinkState.CONNECTED:
            self.btn_toggle_conn.config(text="⏏ Disconnect        ", bg="#d32f2f")
        elif self.is_connecting or conn_state in [LinkState.CONNECTING, LinkState.RECONNECTING]:
            self.btn_toggle_conn.config(text="⏳ Connecting...     ", bg=ACCENT_YELLOW)
        else:
            self.btn_toggle_conn.config(text="🔌 Connect        ", bg="#00897b")

        btn_bg = self.btn_toggle_conn.cget("bg")
        dot_color = ACCENT_GREEN if self.auto_connect else ACCENT_RED
        self.btn_auto_dot.config(fg=dot_color, bg=btn_bg)