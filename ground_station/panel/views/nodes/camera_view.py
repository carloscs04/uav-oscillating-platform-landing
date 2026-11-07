# ground_station/panel/views/nodes/camera_view.py

import base64
import io
import tkinter as tk
from ground_station.utils.colors import (
    BG_CARD, BG_DARK, FG_CYAN, FG_WHITE, ACCENT_GREEN, ACCENT_RED,
    COLOR_PLOT_AXIS, COLOR_PLOT_TEXT
)
from ground_station.receivers.nodes.camera_rx import CameraReceiver

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


class CameraView(tk.LabelFrame):
    """Live Video Stream & ArUco Target Tracking View."""

    def __init__(self, parent, camera_rx=None, **kwargs):
        super().__init__(
            parent, text=" Vision & ArUco Perception ", font=("Helvetica", 10, "bold"),
            fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=6, padx=6, **kwargs
        )
        self.camera_rx = camera_rx

        # 1. Header Metrics Bar
        self.hdr_frame = tk.Frame(self, bg=BG_CARD)
        self.hdr_frame.pack(fill="x", pady=(0, 4))

        self.lbl_target = tk.Label(
            self.hdr_frame, text="TARGET: NO ARUCO DETECTED",
            font=("Consolas", 9, "bold"), fg=ACCENT_RED, bg=BG_CARD
        )
        self.lbl_target.pack(side="left")

        self.lbl_coords = tk.Label(
            self.hdr_frame, text="X: -- | Y: -- | Z: -- | Dist: -- m",
            font=("Consolas", 9, "bold"), fg=FG_WHITE, bg=BG_CARD
        )
        self.lbl_coords.pack(side="right")

        # 2. Video Canvas Box
        self.canvas = tk.Canvas(
            self, bg=BG_DARK, height=260, highlightthickness=1, highlightbackground=COLOR_PLOT_AXIS
        )
        self.canvas.pack(fill="both", expand=True)

        self._image_ref = None
        self._start_loop()

    def _start_loop(self):
        self.update_view()
        self.after(66, self._start_loop)  # ~15 FPS redraw loop

    def update_view(self):
        rx = self.camera_rx or getattr(CameraReceiver, "_active_instance", None)
        if not rx:
            return

        # 1. Fetch ArUco Target Coordinates
        pose = rx.get_packet("aruco_pose", {"detected": False, "x": 0.0, "y": 0.0, "z": 0.0, "id": -1})
        if pose.get("detected", False):
            x, y, z = pose.get("x", 0.0), pose.get("y", 0.0), pose.get("z", 0.0)
            dist = rx.get_aruco_distance()
            self.lbl_target.config(text=f"TARGET: ARUCO ID {pose.get('id', 0)} DETECTED", fg=ACCENT_GREEN)
            self.lbl_coords.config(text=f"X: {x:+.2f} | Y: {y:+.2f} | Z: {z:+.2f} | Dist: {dist:.2f} m")
        else:
            self.lbl_target.config(text="TARGET: NO ARUCO DETECTED", fg=ACCENT_RED)
            self.lbl_coords.config(text="X: -- | Y: -- | Z: -- | Dist: -- m")

        # 2. Render Video Frame Payload
        video_pkt = rx.get_packet("video_frame", {})
        raw_b64 = video_pkt.get("data", "")

        if raw_b64 and HAS_PIL:
            try:
                img_bytes = base64.b64decode(raw_b64)
                img = Image.open(io.BytesIO(img_bytes))
                
                # Resize image dynamically to fit canvas bounds
                cw = self.canvas.winfo_width() or 320
                ch = self.canvas.winfo_height() or 240
                img = img.resize((cw, ch), Image.Resampling.LANCZOS)
                
                self._image_ref = ImageTk.PhotoImage(img)
                self.canvas.create_image(0, 0, image=self._image_ref, anchor="nw")
            except Exception:
                pass
        elif not raw_b64:
            self.canvas.delete("all")
            cw = self.canvas.winfo_width() / 2
            ch = self.canvas.winfo_height() / 2
            self.canvas.create_text(
                cw, ch, text="[NO LIVE VIDEO STREAM]",
                fill=COLOR_PLOT_TEXT, font=("Consolas", 10, "bold")
            )