# ground_station/panel/views/control_view.py

import tkinter as tk
from collections import deque

BG_CARD = "#102a45"
FG_CYAN = "#00f0ff"
FG_BLUE = "#4a9eff"
FG_WHITE = "#ffffff"
COLOR_REF = "#ff5252"      # Target Setpoint Line (Red)
COLOR_ACT = "#00e676"      # Actual Position Line (Green)

class ControlView3Axis(tk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text=" Flight Control Node (3-Axis X, Y, Z Setpoint Tracker) ",
                         font=("Helvetica", 10, "bold"), fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=6)

        self.max_history = 40
        self.history = {
            "X": {"ref": deque([0.0]*self.max_history, maxlen=self.max_history), "act": deque([0.0]*self.max_history, maxlen=self.max_history)},
            "Y": {"ref": deque([0.0]*self.max_history, maxlen=self.max_history), "act": deque([0.0]*self.max_history, maxlen=self.max_history)},
            "Z": {"ref": deque([1.5]*self.max_history, maxlen=self.max_history), "act": deque([1.48]*self.max_history, maxlen=self.max_history)}
        }

        self.canvases = {}
        self.value_labels = {}

        for axis in ["X", "Y", "Z"]:
            frame = tk.Frame(self, bg=BG_CARD)
            frame.pack(fill="x", padx=8, pady=4)

            # Axis Header & Values
            hdr_frame = tk.Frame(frame, bg=BG_CARD)
            hdr_frame.pack(fill="x")
            
            title = tk.Label(hdr_frame, text=f"{axis}-Axis Tracking", font=("Consolas", 9, "bold"), fg=FG_WHITE, bg=BG_CARD)
            title.pack(side="left")

            v_lbl = tk.Label(hdr_frame, text="Ref: 0.00m | Act: 0.00m", font=("Consolas", 8), fg=FG_CYAN, bg=BG_CARD)
            v_lbl.pack(side="right")
            self.value_labels[axis] = v_lbl

            # Increased Canvas height to 75px for larger visual tracking
            canvas = tk.Canvas(frame, bg="#040b14", height=75, highlightthickness=1, highlightbackground=FG_BLUE)
            canvas.pack(fill="x", expand=True, pady=(3, 6))
            self.canvases[axis] = canvas

    def update_position(self, pos_ref: list, pos_act: list):
        axes = ["X", "Y", "Z"]
        for idx, axis in enumerate(axes):
            r_val = pos_ref[idx]
            a_val = pos_act[idx]

            self.history[axis]["ref"].append(r_val)
            self.history[axis]["act"].append(a_val)

            self.value_labels[axis].config(text=f"Ref: {r_val:5.2f}m | Act: {a_val:5.2f}m")
            self._draw_axis_graph(axis)

    def _draw_axis_graph(self, axis: str):
        canvas = self.canvases[axis]
        canvas.delete("all")

        w = canvas.winfo_width() or 300
        h = canvas.winfo_height() or 75

        if w < 10:
            return

        ref_data = list(self.history[axis]["ref"])
        act_data = list(self.history[axis]["act"])

        min_val = min(min(ref_data), min(act_data)) - 0.2
        max_val = max(max(ref_data), max(act_data)) + 0.2
        if max_val - min_val < 0.4:
            max_val += 0.2
            min_val -= 0.2

        range_val = max_val - min_val
        dx = w / (self.max_history - 1)

        # Center Grid Baseline
        zero_y = h - ((0.0 - min_val) / range_val * h)
        if 0 <= zero_y <= h:
            canvas.create_line(0, zero_y, w, zero_y, fill="#1c3b5e", dash=(2, 4))

        # Target Reference Trajectory (Dotted Red)
        ref_pts = []
        for i, val in enumerate(ref_data):
            x = i * dx
            y = h - ((val - min_val) / range_val * h)
            ref_pts.extend([x, y])

        if len(ref_pts) >= 4:
            canvas.create_line(ref_pts, fill=COLOR_REF, width=1.5, dash=(3, 2))

        # Actual Drone Telemetry (Solid Green)
        act_pts = []
        for i, val in enumerate(act_data):
            x = i * dx
            y = h - ((val - min_val) / range_val * h)
            act_pts.extend([x, y])

        if len(act_pts) >= 4:
            canvas.create_line(act_pts, fill=COLOR_ACT, width=2)