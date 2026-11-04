import os
import sys
import argparse
import tkinter as tk
from tkinter import ttk, messagebox

from ground_station.comms.pi_bridge import PiLink
from ground_station.utils.yaml_loader import load_yaml
from ground_station.panel.views.control_view import ControlView3Axis
from ground_station.panel.views.terminal_view import EmbeddedTerminalView

BG_DARK = "#091428"
BG_CARD = "#102a45"
FG_CYAN = "#00f0ff"
FG_BLUE = "#4a9eff"
FG_WHITE = "#ffffff"
ACCENT_GREEN = "#00e676"
ACCENT_RED = "#ff5252"

class MasterPanelGUI:
    def __init__(self, mode: str = "school"):
        self.config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.link = PiLink(self.config_dir, mode=mode)
        
        profiles_path = os.path.join(self.config_dir, "profiles.yaml")
        profiles_data = load_yaml(profiles_path)
        self.available_profiles = list(profiles_data.get("profiles", {}).keys()) or ["school", "usb", "ap", "auto"]

        self.root = tk.Tk()
        self.root.title("Drone Ground Station - Master Panel")
        self.root.geometry("1380x920")
        self.root.configure(bg=BG_DARK)
        self.root.minsize(1150, 750)

        self.node_states = {
            "PX4-MAVLink Node": {"status": "ONLINE", "port": 5005, "rate": "50Hz"},
            "Distance-Sensor Node": {"status": "ONLINE", "port": 5008, "rate": "20Hz"},
            "Aruco-Perception Node": {"status": "STANDBY", "port": 8080, "rate": "30fps"},
            "Bridge/WiFi Node": {"status": "ONLINE", "port": 5006, "rate": "100Hz"}
        }
        self.pos_ref = [0.00, 0.00, 1.50]
        self.pos_act = [0.08, -0.04, 1.48]

        self._build_top_header()
        self._build_main_grid()

        self.link.start()
        self._refresh_telemetry_loop()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_top_header(self):
        header_frame = tk.Frame(self.root, bg=BG_CARD, bd=1, relief="solid", pady=8, padx=15)
        header_frame.pack(fill="x", padx=10, pady=10)

        title_lbl = tk.Label(header_frame, text="GROUND STATION MASTER PANEL",
                             font=("Helvetica", 14, "bold"), fg=FG_WHITE, bg=BG_CARD)
        title_lbl.pack(side="left")

        conn_frame = tk.Frame(header_frame, bg=BG_CARD)
        conn_frame.pack(side="right")

        tk.Label(conn_frame, text="Profile:", font=("Helvetica", 9, "bold"), fg=FG_CYAN, bg=BG_CARD).pack(side="left", padx=4)

        self.selected_profile = tk.StringVar(value=self.link.active_mode)
        profile_dropdown = ttk.OptionMenu(conn_frame, self.selected_profile, self.link.active_mode, *self.available_profiles)
        profile_dropdown.pack(side="left", padx=5)

        btn_connect = tk.Button(conn_frame, text="🔌 Connect", bg="#00897b", fg="white",
                                font=("Helvetica", 9, "bold"), command=self.cmd_connect_profile)
        btn_connect.pack(side="left", padx=5)

        self.status_bar = tk.Label(header_frame, text="Initializing network link...",
                                   font=("Consolas", 10), fg=FG_BLUE, bg=BG_CARD)
        self.status_bar.pack(side="left", padx=20)

    def _build_main_grid(self):
        main_frame = tk.Frame(self.root, bg=BG_DARK)
        main_frame.pack(fill="both", expand=True, padx=10, pady=0)

        # LEFT COLUMN (Camera + Left-Aligned Buttons)
        left_col = tk.Frame(main_frame, bg=BG_DARK)
        left_col.pack(side="left", fill="both", expand=True, padx=(0, 5))

        cam_box = tk.LabelFrame(left_col, text=" Live Camera Feed ", font=("Helvetica", 10, "bold"),
                                fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove")
        cam_box.pack(fill="both", expand=True, pady=(0, 5))

        cam_canvas = tk.Canvas(cam_box, bg="#040b14", highlightthickness=0)
        cam_canvas.pack(fill="both", expand=True, padx=8, pady=8)
        cam_canvas.create_text(300, 220, text="[ LIVE MJPEG VIDEO STREAM ]\n(Port 8080)",
                                fill=FG_CYAN, font=("Consolas", 12, "bold"))

        # Operations & Action Controls (Anchored West / Left Aligned)
        action_box = tk.LabelFrame(left_col, text=" Operations & Action Controls ", font=("Helvetica", 10, "bold"),
                                   fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=8)
        action_box.pack(fill="x", pady=(5, 0))

        btn_grid = tk.Frame(action_box, bg=BG_CARD)
        btn_grid.pack(anchor="w", padx=10)

        b_style = {"font": ("Helvetica", 9, "bold"), "bd": 0, "padx": 14, "pady": 6, "cursor": "hand2"}

        tk.Button(btn_grid, text="⚡ ARM DRONE", bg="#d84315", fg="white", command=lambda: self.terminal_view.log_message("[TX] ARM DRONE packet sent."), **b_style).grid(row=0, column=0, padx=(0, 8))
        tk.Button(btn_grid, text="▶ START MISSION", bg="#00897b", fg="white", command=lambda: self.terminal_view.log_message("[TX] START MISSION packet sent."), **b_style).grid(row=0, column=1, padx=8)
        tk.Button(btn_grid, text="🛬 LAND DRONE", bg="#0288d1", fg="white", command=lambda: self.terminal_view.log_message("[TX] LAND DRONE packet sent."), **b_style).grid(row=0, column=2, padx=8)
        tk.Button(btn_grid, text="⚠ ABORT", bg="#c62828", fg="white", command=lambda: self.terminal_view.log_message("[EMERGENCY] ABORT MISSION SENT!"), **b_style).grid(row=0, column=3, padx=8)

        # RIGHT COLUMN (Master Node + Nodes Health + 3-Axis Graphics + Console Terminal)
        right_col = tk.Frame(main_frame, bg=BG_DARK, width=540)
        right_col.pack(side="right", fill="both", expand=False, padx=(5, 0))

        # 1. System Master Node
        master_box = tk.LabelFrame(right_col, text=" System Master Node ", font=("Helvetica", 10, "bold"),
                                   fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=4)
        master_box.pack(fill="x", pady=(0, 4))

        tk.Label(master_box, text="Active Stage : STAGE 3 (LAUNCH MISSION)", font=("Consolas", 9, "bold"), fg=FG_WHITE, bg=BG_CARD, anchor="w").pack(fill="x", padx=8)
        tk.Label(master_box, text="Master State : [ ACTIVE ]", font=("Consolas", 9), fg=ACCENT_GREEN, bg=BG_CARD, anchor="w").pack(fill="x", padx=8)

        # 2. Sub-Nodes Health Matrix
        nodes_box = tk.LabelFrame(right_col, text=" Sub-Nodes Health Matrix ", font=("Helvetica", 10, "bold"),
                                  fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=4)
        nodes_box.pack(fill="x", pady=4)

        for name, info in self.node_states.items():
            row = tk.Frame(nodes_box, bg=BG_CARD)
            row.pack(fill="x", padx=8, pady=2)
            
            tk.Label(row, text=f"• {name}", font=("Consolas", 9), fg=FG_BLUE, bg=BG_CARD, anchor="w").pack(side="left")

            right_subframe = tk.Frame(row, bg=BG_CARD)
            right_subframe.pack(side="right")

            tk.Label(right_subframe, text=f"[{info['status']}]", font=("Consolas", 9, "bold"), fg=ACCENT_GREEN, bg=BG_CARD).pack(side="left", padx=(0, 8))

            btn_inspect = tk.Button(right_subframe, text="Inspect Data", bg="#3949ab", fg="white", font=("Helvetica", 8, "bold"),
                                    command=lambda n=name: self.open_node_inspector(n))
            btn_inspect.pack(side="right")

        # 3. Flight Control Node (3-Axis X, Y, Z Line Graphs)
        self.control_plot = ControlView3Axis(right_col)
        self.control_plot.pack(fill="x", pady=4)

        # 4. Interactive GCS Terminal Console
        self.terminal_view = EmbeddedTerminalView(right_col, on_execute_command=self.handle_cli_command)
        self.terminal_view.pack(fill="both", expand=True, pady=(4, 0))

    def open_node_inspector(self, node_name: str):
        win = tk.Toplevel(self.root)
        win.title(f"Node Inspector - {node_name}")
        win.geometry("550x320")
        win.configure(bg=BG_DARK)

        info = self.node_states.get(node_name, {})
        hdr = tk.Label(win, text=f"LIVE TELEMETRY STREAM: {node_name}", font=("Helvetica", 11, "bold"), fg=FG_CYAN, bg=BG_CARD, pady=6)
        hdr.pack(fill="x")

        txt = tk.Text(win, bg="#040b14", fg=ACCENT_GREEN, font=("Consolas", 9), bd=0)
        txt.pack(fill="both", expand=True, padx=10, pady=10)
        txt.insert("end", f"Connecting to {node_name} on UDP Port {info.get('port')}...\n")
        txt.insert("end", f"Data Stream Rate: {info.get('rate')}\n")
        txt.insert("end", f"---------------------------------------------------\n")
        txt.insert("end", f"[0.001s] HEARTBEAT_OK | status_flag=0x01\n")
        txt.insert("end", f"[0.020s] PACKET_RX payload_size=64 bytes\n")

    def handle_cli_command(self, cmd: str):
        if cmd.lower() == "help":
            self.terminal_view.log_message("Available commands: start, land, abort, clear, status")
        elif cmd.lower() == "clear":
            self.terminal_view.log_area.delete("1.0", "end")
        else:
            self.terminal_view.log_message(f"[EXEC] Processed: {cmd}")

    def cmd_connect_profile(self):
        new_mode = self.selected_profile.get()
        self.link.switch_mode(new_mode)
        self.terminal_view.log_message(f"[NET] Switching connection profile to '{new_mode}'...")

    def _refresh_telemetry_loop(self):
        health = self.link.get_health()
        self.status_bar.config(
            text=f"Mode: [{health['mode'].upper()}] | IP: {health['ip']} | Link: [{health['state']}] | Latency: {health['latency_ms']}ms"
        )
        self.control_plot.update_position(self.pos_ref, self.pos_act)
        self.root.after(200, self._refresh_telemetry_loop)

    def on_close(self):
        self.link.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GCS Master Panel Desktop GUI")
    parser.add_argument("--mode", default="school", choices=["school", "usb", "ap", "auto"])
    args = parser.parse_args()

    app = MasterPanelGUI(mode=args.mode)
    app.run()