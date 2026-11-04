# ground_station/panel/master_panel.py

import os
import sys

PANEL_DIR = os.path.dirname(os.path.abspath(__file__))
GCS_ROOT = os.path.abspath(os.path.join(PANEL_DIR, ".."))
CODEBASE_ROOT = os.path.abspath(os.path.join(GCS_ROOT, ".."))

for path in (GCS_ROOT, CODEBASE_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

import argparse
import tkinter as tk
from tkinter import ttk, messagebox

from ground_station.utils.yaml_loader import load_yaml
from ground_station.panel.views.control_view import ControlView3Axis
from ground_station.panel.features.panel_controller import PanelController

# Dark Blue Palette
BG_DARK = "#091428"
BG_CARD = "#102a45"
FG_CYAN = "#00f0ff"
FG_BLUE = "#4a9eff"
FG_WHITE = "#ffffff"
ACCENT_GREEN = "#00e676"
ACCENT_RED = "#ff5252"
ACCENT_YELLOW = "#ffb300"

class MasterPanelGUI:
    def __init__(self, mode: str = None):
        self.config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.controller = PanelController(self.config_dir, initial_mode=mode)
        self.controller.set_log_callback(self.append_master_console_log)

        # Tkinter Window Initialization
        self.root = tk.Tk()
        self.root.title("Drone Ground Station - Master Panel")
        self.root.geometry("1380x920")
        self.root.configure(bg=BG_DARK)
        self.root.minsize(1150, 750)

        self._build_top_header()
        self._build_main_grid()

        self.controller.start()
        self._refresh_telemetry_loop()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_top_header(self):
        header_frame = tk.Frame(self.root, bg=BG_CARD, bd=1, relief="solid", pady=8, padx=15)
        header_frame.pack(fill="x", padx=10, pady=10)

        title_lbl = tk.Label(header_frame, text="GROUND STATION MASTER PANEL",
                             font=("Helvetica", 14, "bold"), fg=FG_WHITE, bg=BG_CARD)
        title_lbl.pack(side="left")

        # Connection Control Bar
        conn_frame = tk.Frame(header_frame, bg=BG_CARD)
        conn_frame.pack(side="right")

        tk.Label(conn_frame, text="Profile:", font=("Helvetica", 9, "bold"), fg=FG_CYAN, bg=BG_CARD).pack(side="left", padx=4)

        # Profile Dropdown
        self.selected_profile = tk.StringVar(value=self.controller.link.active_mode)
        profile_dropdown = ttk.OptionMenu(
            conn_frame,
            self.selected_profile,
            self.controller.link.active_mode,
            *self.controller.available_profiles,
            command=self.on_profile_dropdown_change
        )
        profile_dropdown.pack(side="left", padx=4)

        # Main Button Container
        self.conn_box = tk.Frame(conn_frame, bg=BG_CARD)
        self.conn_box.pack(side="left", padx=6)

        # Connect / Disconnect Action Button
        self.btn_toggle_conn = tk.Button(
            self.conn_box,
            text="🔌 Connect        ",
            bg="#00897b",
            fg="white",
            font=("Helvetica", 9, "bold"),
            padx=14,
            pady=4,
            bd=0,
            relief="flat",
            cursor="hand2",
            command=self.cmd_toggle_connection
        )
        self.btn_toggle_conn.pack(fill="both", expand=True)

        # Borderless LED Dot (●)
        self.btn_auto_dot = tk.Label(
            self.btn_toggle_conn,
            text="●",
            font=("Helvetica", 8, "bold"),
            bg="#00897b",
            fg=ACCENT_GREEN,
            bd=0,
            relief="flat",
            highlightthickness=0,
            cursor="hand2"
        )
        self.btn_auto_dot.place(relx=1.0, rely=0.0, x=-4, y=2, anchor="ne")
        self.btn_auto_dot.bind("<Button-1>", lambda e: self.cmd_toggle_auto_reconnect())

        self.status_bar = tk.Label(header_frame, text="Initializing network link...",
                                   font=("Consolas", 10), fg=FG_BLUE, bg=BG_CARD)
        self.status_bar.pack(side="left", padx=15)

    def _build_main_grid(self):
        main_frame = tk.Frame(self.root, bg=BG_DARK)
        main_frame.pack(fill="both", expand=True, padx=10, pady=0)

        # LEFT COLUMN (Live Camera Feed + Action Controls)
        left_col = tk.Frame(main_frame, bg=BG_DARK)
        left_col.pack(side="left", fill="both", expand=True, padx=(0, 5))

        cam_box = tk.LabelFrame(left_col, text=" Live Camera Feed ", font=("Helvetica", 10, "bold"),
                                fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove")
        cam_box.pack(fill="both", expand=True, pady=(0, 5))

        cam_canvas = tk.Canvas(cam_box, bg="#040b14", highlightthickness=0)
        cam_canvas.pack(fill="both", expand=True, padx=8, pady=8)
        cam_canvas.create_text(300, 220, text="[ LIVE MJPEG VIDEO STREAM ]\n(Port 8080)",
                                fill=FG_CYAN, font=("Consolas", 12, "bold"))

        # Operations & Action Controls
        action_box = tk.LabelFrame(left_col, text=" Operations & Action Controls ", font=("Helvetica", 10, "bold"),
                                   fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=8)
        action_box.pack(fill="x", pady=(5, 0))

        btn_grid = tk.Frame(action_box, bg=BG_CARD)
        btn_grid.pack(anchor="w", padx=10)

        b_style = {"font": ("Helvetica", 9, "bold"), "bd": 0, "padx": 14, "pady": 6, "cursor": "hand2"}

        tk.Button(btn_grid, text="⚡ ARM DRONE", bg="#d84315", fg="white", command=self.cmd_arm, **b_style).grid(row=0, column=0, padx=(0, 8))
        tk.Button(btn_grid, text="▶ START MISSION", bg="#00897b", fg="white", command=self.cmd_start_mission, **b_style).grid(row=0, column=1, padx=8)
        tk.Button(btn_grid, text="🛬 LAND DRONE", bg="#0288d1", fg="white", command=self.cmd_land_drone, **b_style).grid(row=0, column=2, padx=8)
        tk.Button(btn_grid, text="⚠ ABORT", bg="#c62828", fg="white", command=self.cmd_abort, **b_style).grid(row=0, column=3, padx=8)

        # RIGHT COLUMN (Master Node + Sub-Nodes + 3-Axis Tracker + Master Node Console)
        right_col = tk.Frame(main_frame, bg=BG_DARK, width=540)
        right_col.pack(side="right", fill="both", expand=False, padx=(5, 0))

        # 1. System Master Node Box
        master_box = tk.LabelFrame(right_col, text=" System Master Node ", font=("Helvetica", 10, "bold"),
                                   fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=4)
        master_box.pack(fill="x", pady=(0, 4))

        self.lbl_stage = tk.Label(master_box, text=f"Active Stage : {self.controller.current_stage}", font=("Consolas", 9, "bold"), fg=FG_WHITE, bg=BG_CARD, anchor="w")
        self.lbl_stage.pack(fill="x", padx=8)
        
        self.lbl_master_state = tk.Label(master_box, text="Master State : [ DISCONNECTED ]", font=("Consolas", 9), fg=ACCENT_RED, bg=BG_CARD, anchor="w")
        self.lbl_master_state.pack(fill="x", padx=8)

        # 2. Sub-Nodes Health Matrix
        nodes_box = tk.LabelFrame(right_col, text=" Sub-Nodes Health Matrix ", font=("Helvetica", 10, "bold"),
                                  fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=4)
        nodes_box.pack(fill="x", pady=4)

        self.status_widgets = {}
        for name, info in self.controller.node_states.items():
            row = tk.Frame(nodes_box, bg=BG_CARD)
            row.pack(fill="x", padx=8, pady=2)
            
            tk.Label(row, text=f"• {name}", font=("Consolas", 9), fg=FG_BLUE, bg=BG_CARD, anchor="w").pack(side="left")

            right_subframe = tk.Frame(row, bg=BG_CARD)
            right_subframe.pack(side="right")

            lbl_stat = tk.Label(right_subframe, text=f"[{info['status']}]", font=("Consolas", 9, "bold"), fg=ACCENT_RED, bg=BG_CARD)
            lbl_stat.pack(side="left", padx=(0, 8))
            self.status_widgets[name] = lbl_stat

            btn_inspect = tk.Button(right_subframe, text="Inspect Data", bg="#3949ab", fg="white", font=("Helvetica", 8, "bold"),
                                    command=lambda n=name: self.open_node_inspector(n))
            btn_inspect.pack(side="right")

        # 3. 3-Axis Tracker Line Graphs
        self.control_plot = ControlView3Axis(right_col)
        self.control_plot.pack(fill="x", pady=4)

        # 4. Master Node Console + Command Input
        console_box = tk.LabelFrame(right_col, text=" Master Node Stream & Command Console ", font=("Helvetica", 10, "bold"),
                                   fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove")
        console_box.pack(fill="both", expand=True, pady=(4, 0))

        self.console_log = tk.Text(console_box, bg="#040b14", fg=FG_CYAN, font=("Consolas", 9),
                                   height=6, wrap="word", bd=0, highlightthickness=0)
        self.console_log.pack(fill="both", expand=True, padx=6, pady=5)

        input_frame = tk.Frame(console_box, bg=BG_CARD)
        input_frame.pack(fill="x", padx=6, pady=(0, 6))

        tk.Label(input_frame, text="GCS> ", font=("Consolas", 10, "bold"), fg=FG_WHITE, bg=BG_CARD).pack(side="left")

        self.cmd_entry = tk.Entry(input_frame, bg="#040b14", fg=FG_WHITE, insertbackground=FG_WHITE,
                                  font=("Consolas", 10), bd=1, relief="solid")
        self.cmd_entry.pack(side="left", fill="x", expand=True, padx=5)
        self.cmd_entry.bind("<Return>", self._on_send_command)

        btn_send = tk.Button(input_frame, text="Send", bg="#00897b", fg="white",
                             font=("Helvetica", 9, "bold"), command=self._on_send_command)
        btn_send.pack(side="right")

    def on_profile_dropdown_change(self, new_mode: str):
        """Respects Auto-Reconnect toggle when switching profiles from the dropdown."""
        self.controller.select_profile(new_mode)

    def append_master_console_log(self, text: str):
        self.console_log.insert("end", f"{text}\n")
        self.console_log.see("end")

    def _on_send_command(self, event=None):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self.controller.send_cli_command(cmd)
            self.cmd_entry.delete(0, "end")

    def open_node_inspector(self, node_name: str):
        win = tk.Toplevel(self.root)
        win.title(f"Node Inspector - {node_name}")
        win.geometry("550x320")
        win.configure(bg=BG_DARK)

        info = self.controller.node_states.get(node_name, {})
        hdr = tk.Label(win, text=f"LIVE TELEMETRY STREAM: {node_name}", font=("Helvetica", 11, "bold"), fg=FG_CYAN, bg=BG_CARD, pady=6)
        hdr.pack(fill="x")

        txt = tk.Text(win, bg="#040b14", fg=ACCENT_GREEN, font=("Consolas", 9), bd=0)
        txt.pack(fill="both", expand=True, padx=10, pady=10)
        txt.insert("end", f"Connecting to {node_name} on UDP Port {info.get('port')}...\n")
        txt.insert("end", f"Data Stream Rate: {info.get('rate')}\n")
        txt.insert("end", f"Current Node Health Status: [{info.get('status')}]\n")
        txt.insert("end", f"---------------------------------------------------\n")

    def cmd_toggle_connection(self):
        if self.controller.is_link_connected():
            ok, msg = self.controller.disconnect_wifi()
            if not ok:
                messagebox.showerror("Wi-Fi Safety Guard", msg)
        else:
            new_mode = self.selected_profile.get()
            self.controller.connect_profile(new_mode)

    def cmd_toggle_auto_reconnect(self):
        ok, msg = self.controller.toggle_auto_reconnect()
        if not ok:
            messagebox.showwarning("Safety Guard", msg)
        self._update_auto_dot_ui()

    def _update_auto_dot_ui(self):
        """Seamlessly updates dot color matching the parent button background."""
        btn_bg = self.btn_toggle_conn.cget("bg")
        dot_color = ACCENT_GREEN if self.controller.auto_reconnect else ACCENT_RED
        self.btn_auto_dot.config(
            fg=dot_color,
            bg=btn_bg
        )

    def cmd_arm(self):
        self.controller.is_armed = True
        self.controller.auto_reconnect = True  # Force Auto-Reconnect ON
        self._update_auto_dot_ui()
        self.append_master_console_log("[TX] ARM DRONE packet sent to Master Node. Drone ARMED.")

    def cmd_start_mission(self):
        if not self.controller.is_armed:
            messagebox.showwarning("Arming Guard", "Must ARM drone before starting mission!")
            return
        self.controller.in_flight = True
        self.controller.auto_reconnect = True  # Force Auto-Reconnect ON
        self._update_auto_dot_ui()
        self.controller.current_stage = "STAGE 3 (LAUNCH MISSION)"
        self.lbl_stage.config(text=f"Active Stage : {self.controller.current_stage}")
        self.append_master_console_log("[TX] START MISSION packet sent.")

    def cmd_land_drone(self):
        self.controller.in_flight = False
        self.controller.is_armed = False
        self.controller.current_stage = "STAGE 5 (LANDED / DISARMED)"
        self.lbl_stage.config(text=f"Active Stage : {self.controller.current_stage}")
        self.append_master_console_log("[TX] LAND DRONE packet sent.")

    def cmd_abort(self):
        self.controller.in_flight = False
        self.controller.is_armed = False
        self.controller.current_stage = "EMERGENCY ABORT"
        self.lbl_stage.config(text=f"Active Stage : {self.controller.current_stage}")
        self.append_master_console_log("[EMERGENCY] ABORT MISSION SENT!")

    def _refresh_telemetry_loop(self):
        self.controller.update_telemetry_from_pi()
        health = self.controller.get_link_health()
        link_state = health.get("state", "OFFLINE")

        # Header Latency Display
        lat = health.get("latency_ms")
        lat_str = f"{lat:.1f}ms" if isinstance(lat, (int, float)) and self.controller.is_link_connected() else "N/A ms"
        
        self.status_bar.config(
            text=f"Mode: [{health['mode'].upper()}] | IP: {health['ip']} | Link: [{link_state}] | Latency: {lat_str}"
        )

        # Dynamic Connection Button Styling
        if self.controller.is_link_connected():
            self.btn_toggle_conn.config(text="⏏ Disconnect        ", bg="#d32f2f")
        elif self.controller.is_connecting or link_state in ["CONNECTING", "RECONNECTING"]:
            self.btn_toggle_conn.config(text="⏳ Connecting...     ", bg=ACCENT_YELLOW)
        else:
            self.btn_toggle_conn.config(text="🔌 Connect        ", bg="#00897b")

        # Refresh LED dot color & parent button background matching
        self._update_auto_dot_ui()

        # System Master Node Badge Colors
        m_state = self.controller.master_state
        m_color = ACCENT_GREEN if m_state in ["ACTIVE", "OK"] else (ACCENT_YELLOW if m_state in ["POOR", "CONNECTING"] else ACCENT_RED)
        self.lbl_master_state.config(text=f"Master State : [ {m_state} ]", fg=m_color)

        # Sub-Nodes Health Matrix Badges
        for name, info in self.controller.node_states.items():
            if name in self.status_widgets:
                st = info["status"]
                c = ACCENT_GREEN if st in ["OK", "ONLINE"] else (ACCENT_YELLOW if st in ["POOR", "STANDBY"] else ACCENT_RED)
                self.status_widgets[name].config(text=f"[{st}]", fg=c)

        self.control_plot.update_position(self.controller.pos_ref, self.controller.pos_act)
        self.root.after(200, self._refresh_telemetry_loop)

    def on_close(self):
        self.controller.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config"))
    profiles_data = load_yaml(os.path.join(config_dir, "profiles.yaml"))
    valid_profiles = list(profiles_data.get("profiles", {}).keys()) or ["wifi"]

    parser = argparse.ArgumentParser(description="GCS Master Panel Desktop GUI")
    parser.add_argument("--mode", default=valid_profiles[0], choices=valid_profiles)
    args = parser.parse_args()

    app = MasterPanelGUI(mode=args.mode)
    app.run()