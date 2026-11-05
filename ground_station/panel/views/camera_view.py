# ground_station/panel/views/camera_view.py

import tkinter as tk
from ground_station.utils.colors import BG_CARD, FG_CYAN

class CameraView(tk.LabelFrame):
    """Live MJPEG Video Stream Canvas container."""
    
    def __init__(self, parent, **kwargs):
        super().__init__(parent, text=" Live Camera Feed ", font=("Helvetica", 10, "bold"),
                         fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", **kwargs)
        
        self.canvas = tk.Canvas(self, bg="#040b14", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=8, pady=8)
        self.canvas.create_text(300, 220, text="[ LIVE MJPEG VIDEO STREAM ]\n(Port 8080)",
                                fill=FG_CYAN, font=("Consolas", 12, "bold"))