# File: ground_station/panel/views/nodes/camera_view.py

import base64
import io
import tkinter as tk
from ground_station.utils.colors import (
    BG_CARD, BG_DARK, FG_CYAN, FG_WHITE, ACCENT_GREEN, ACCENT_RED, ACCENT_YELLOW,
    COLOR_PLOT_AXIS, COLOR_PLOT_TEXT
)
from ground_station.receivers.nodes.camera_rx import CameraReceiver
from ground_station.receivers.nodes.vision_rx import VisionReceiver
from ground_station.receivers.nodes.control_rx import ControlReceiver
from ground_station.receivers.nodes.mavlink_rx import MavlinkReceiver
from ground_station.comms.pi_bridge import PiLink, LinkState

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


class CameraView(tk.LabelFrame):
    """Live Video Stream & ArUco Target Tracking View."""

    def __init__(self, parent, camera_rx=None, vision_rx=None, **kwargs):
        super().__init__(
            parent, text=" Vision & ArUco Perception ", font=("Helvetica", 10, "bold"),
            fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=6, padx=6, **kwargs
        )
        self.camera_rx = camera_rx
        self.vision_rx = vision_rx

        # 1. Metrics Header
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

        # 2. Video Canvas
        self.canvas = tk.Canvas(
            self, bg=BG_DARK, height=260, highlightthickness=1, highlightbackground=COLOR_PLOT_AXIS
        )
        self.canvas.pack(fill="both", expand=True)

        self._image_ref = None
        self._start_loop()

    def _start_loop(self):
        self.update_view()
        self.after(66, self._start_loop)  # Redraw at ~15 FPS

    def update_view(self):
        cam_rx = self.camera_rx or getattr(CameraReceiver, "_active_instance", None)
        vis_rx = self.vision_rx or getattr(VisionReceiver, "_active_instance", None)
        ctrl_rx = getattr(ControlReceiver, "_active_instance", None)

        # 1. ArUco Coordinates (VisionReceiver on port 5006, with ControlReceiver fallback)
        pose = {"detected": False, "x": 0.0, "y": 0.0, "z": 0.0, "id": -1}
        if vis_rx:
            pose = vis_rx.get_packet("aruco_pose", pose)
        if not pose.get("detected", False) and ctrl_rx:
            pose = ctrl_rx.get_packet("aruco_pose", pose)

        if pose.get("detected", False):
            x = float(pose.get("x", 0.0))
            y = float(pose.get("y", 0.0))
            z = float(pose.get("z", 0.0))
            dist = (x**2 + y**2 + z**2) ** 0.5
            target_id = pose.get("id", 0)
            self.lbl_target.config(text=f"TARGET: ARUCO ID {target_id} DETECTED", fg=ACCENT_GREEN)
            self.lbl_coords.config(text=f"X: {x:+.2f} | Y: {y:+.2f} | Z: {z:+.2f} | Dist: {dist:.2f} m")
        else:
            self.lbl_target.config(text="TARGET: NO ARUCO DETECTED", fg=ACCENT_RED)
            self.lbl_coords.config(text="X: -- | Y: -- | Z: -- | Dist: -- m")

        # 2. Render Video Frame (Base64 JPEG from CameraReceiver on port 5003)
        raw_b64 = ""
        if cam_rx:
            video_pkt = cam_rx.get_packet("video_frame", {})
            raw_b64 = video_pkt.get("data", "")

        if raw_b64 and HAS_PIL:
            try:
                img_bytes = base64.b64decode(raw_b64)
                img = Image.open(io.BytesIO(img_bytes))

                cw = self.canvas.winfo_width() or 320
                ch = self.canvas.winfo_height() or 240
                img = img.resize((cw, ch), Image.Resampling.LANCZOS)

                self._image_ref = ImageTk.PhotoImage(img)
                self.canvas.delete("video_layer")
                self.canvas.create_image(0, 0, image=self._image_ref, anchor="nw", tags="video_layer")
            except Exception:
                pass
        elif not raw_b64:
            self.canvas.delete("all")
            cw = (self.canvas.winfo_width() or 320) / 2
            ch = (self.canvas.winfo_height() or 240) / 2
            self.canvas.create_text(
                cw, ch, text="[NO LIVE VIDEO STREAM]",
                fill=COLOR_PLOT_TEXT, font=("Consolas", 10, "bold"), tags="video_layer"
            )

        # 3. Overlay drone battery read from MAVLink
        self._render_battery_overlay()

    def _render_battery_overlay(self):
        self.canvas.delete("bat_overlay")

        active_link = getattr(PiLink, "_active_instance", None)
        if not active_link or not hasattr(active_link, "get_health"):
            return
        if active_link.get_health().get("state") != LinkState.CONNECTED:
            return

        mav_rx = getattr(MavlinkReceiver, "_active_instance", None)
        if not mav_rx or not hasattr(mav_rx, "get_health_info"):
            return

        health_info = mav_rx.get_health_info()
        h_status = health_info.get("status", "OFFLINE")

        if h_status not in ["HEALTHY", "POOR"]:
            return

        cw = self.canvas.winfo_width() or 320
        ch = self.canvas.winfo_height() or 240

        battery_pct = health_info.get("battery_pct", -1)
        voltage_v = health_info.get("voltage_v", 0.0)

        if battery_pct >= 0 and voltage_v > 0:
            bat_str = f"BAT: {battery_pct}% ({voltage_v}V)"
            color = ACCENT_GREEN if battery_pct > 50 else (ACCENT_YELLOW if battery_pct > 20 else ACCENT_RED)
        elif battery_pct >= 0:
            bat_str = f"BAT: {battery_pct}%"
            color = ACCENT_GREEN if battery_pct > 50 else (ACCENT_YELLOW if battery_pct > 20 else ACCENT_RED)
        elif voltage_v > 0:
            bat_str = f"BAT: {voltage_v}V"
            color = ACCENT_GREEN
        else:
            bat_str = "BAT: DISCONNECTED"
            color = ACCENT_RED

        self.canvas.create_text(
            cw - 12, ch - 12, text=bat_str, anchor="se",
            fill=color, font=("Consolas", 9, "bold"), tags="bat_overlay"
        )
        self.canvas.tag_raise("bat_overlay")