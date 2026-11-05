# ground_station/panel/master_panel.py

import os
import sys
import argparse
import tkinter as tk

from ground_station.utils.yaml_loader import load_yaml
from ground_station.utils.colors import BG_DARK
from ground_station.panel.features.panel_controller import PanelController

from ground_station.panel.views.header_view import HeaderView
from ground_station.panel.views.camera_view import CameraView
from ground_station.panel.views.actions_view import ActionsView
from ground_station.panel.views.master_node_view import MasterNodeView
from ground_station.panel.views.node_health_view import NodeHealthView
from ground_station.panel.views.control_view import ControlView3Axis
from ground_station.panel.views.console_view import ConsoleView

class MasterPanelGUI:
    def __init__(self, mode: str = None):
        self.config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config"))
        self.controller = PanelController(self.config_dir, initial_mode=mode)

        self.root = tk.Tk()
        self.root.title("Drone Ground Station - Master Panel")
        self.root.geometry("1380x920")
        self.root.configure(bg=BG_DARK)
        self.root.minsize(1150, 750)

        # Build Modular Views
        self.header_view = HeaderView(self.root, self.controller)
        self.header_view.pack(fill="x", padx=10, pady=10)

        main_frame = tk.Frame(self.root, bg=BG_DARK)
        main_frame.pack(fill="both", expand=True, padx=10, pady=0)

        # Left Column
        left_col = tk.Frame(main_frame, bg=BG_DARK)
        left_col.pack(side="left", fill="both", expand=True, padx=(0, 5))
        
        self.cam_view = CameraView(left_col)
        self.cam_view.pack(fill="both", expand=True, pady=(0, 5))
        
        self.actions_view = ActionsView(left_col, self.controller)
        self.actions_view.pack(fill="x", pady=(5, 0))

        # Right Column
        right_col = tk.Frame(main_frame, bg=BG_DARK, width=540)
        right_col.pack(side="right", fill="both", expand=False, padx=(5, 0))

        self.master_node_view = MasterNodeView(right_col, self.controller)
        self.master_node_view.pack(fill="x", pady=(0, 4))

        self.node_health_view = NodeHealthView(right_col, self.controller)
        self.node_health_view.pack(fill="x", pady=4)

        self.control_plot = ControlView3Axis(right_col)
        self.control_plot.pack(fill="x", pady=4)

        self.console_view = ConsoleView(right_col, self.controller)
        self.console_view.pack(fill="both", expand=True, pady=(4, 0))

        # Bind controller console logger to ConsoleView
        self.controller.set_log_callback(self.console_view.append_log)

        self.controller.start()
        self._refresh_loop()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _refresh_loop(self):
        self.controller.update_telemetry_loop()
        
        health = self.controller.link_mgr.get_health()
        is_connecting = self.controller.link_mgr.is_connecting
        auto_reconnect = self.controller.link_mgr.auto_reconnect
        
        self.header_view.update_status(health, is_connecting, auto_reconnect)
        self.master_node_view.update_view()
        self.node_health_view.update_matrix()
        self.control_plot.update_position(self.controller.pos_ref, self.controller.pos_act)
        
        self.root.after(200, self._refresh_loop)

    def on_close(self):
        self.controller.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config"))
    profiles_data = load_yaml(os.path.join(config_dir, "profiles.yaml"))
    valid_profiles = list(profiles_data.get("profiles", {}).keys())

    parser = argparse.ArgumentParser(description="GCS Master Panel Desktop GUI")
    parser.add_argument("--mode", default=valid_profiles[0], choices=valid_profiles)
    args = parser.parse_args()

    app = MasterPanelGUI(mode=args.mode)
    app.run()