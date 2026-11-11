# File: ground_station/panel/views/nodes/control_view.py

import tkinter as tk
from collections import deque
from ground_station.utils.colors import (
    BG_CARD, BG_DARK, FG_CYAN, FG_WHITE, ACCENT_GREEN, ACCENT_RED,
    BG_PLOT_CANVAS, COLOR_PLOT_REF, COLOR_PLOT_ACT, COLOR_PLOT_GRID,
    COLOR_PLOT_AXIS, COLOR_PLOT_ZERO, COLOR_PLOT_TEXT
)
from ground_station.receivers.nodes.control_rx import ControlReceiver


class ControlView3Axis(tk.LabelFrame):
    """3-Axis (X, Y, Z) Real-Time Trajectory Plotting with MATLAB Dark Grid styling and Inspector Popups."""

    def __init__(self, parent, max_history: int = 80, control_rx=None, **kwargs):
        super().__init__(
            parent, text=" 3-Axis Trajectory Tracking (X, Y, Z) ",
            font=("Helvetica", 10, "bold"), fg=FG_CYAN, bg=BG_CARD,
            bd=2, relief="groove", pady=6, padx=6, **kwargs
        )
        self.max_history = max_history
        self.control_rx = control_rx

        self.history = {
            "X": {"ref": deque([0.0] * max_history, maxlen=max_history),
                  "act": deque([0.0] * max_history, maxlen=max_history)},
            "Y": {"ref": deque([0.0] * max_history, maxlen=max_history),
                  "act": deque([0.0] * max_history, maxlen=max_history)},
            "Z": {"ref": deque([1.5] * max_history, maxlen=max_history),
                  "act": deque([0.0] * max_history, maxlen=max_history)},
        }

        self.canvases = {}
        self.active_popups = {}

        for axis in ["X", "Y", "Z"]:
            frame = tk.Frame(self, bg=BG_CARD)
            frame.pack(fill="x", pady=2)

            lbl = tk.Label(frame, text=f"Axis {axis}", font=("Consolas", 9, "bold"), fg=FG_WHITE, bg=BG_CARD, width=6)
            lbl.pack(side="left")

            canvas = tk.Canvas(frame, bg=BG_PLOT_CANVAS, height=65, highlightthickness=1, highlightbackground=COLOR_PLOT_AXIS)
            canvas.pack(side="left", fill="both", expand=True, padx=4)
            canvas.bind("<Configure>", lambda event, a=axis: self._render_axis_canvas(self.canvases[a], a))
            self.canvases[axis] = canvas

            btn_expand = tk.Button(
                frame, text="🔍 Expand", bg="#3949ab", fg="white",
                font=("Helvetica", 8, "bold"), bd=0, relief="flat", padx=8, pady=2, cursor="hand2",
                command=lambda a=axis: self.open_expanded_axis_view(a)
            )
            btn_expand.pack(side="right", padx=(2, 0))

        # Loop autónomo de redibujo a 20Hz (50ms)
        self._start_auto_update()

    def _start_auto_update(self):
        """Timer loop de 50ms para actualizar telemetría y redibujar."""
        self.update_view()
        self.after(50, self._start_auto_update)

    def open_expanded_axis_view(self, axis: str):
        """Abre ventana modal popup en alta resolución para inspección de eje único."""
        if axis in self.active_popups:
            popup_info = self.active_popups[axis]
            if popup_info["window"].winfo_exists():
                popup_info["window"].lift()
                popup_info["window"].focus_set()
                return

        win = tk.Toplevel(self.winfo_toplevel())
        win.title(f"Axis {axis} Trajectory Inspector")
        win.geometry("720x450")
        win.configure(bg=BG_DARK)
        win.minsize(500, 320)

        hdr = tk.Frame(win, bg=BG_CARD, pady=8, padx=12)
        hdr.pack(fill="x")

        tk.Label(hdr, text=f"AXIS {axis} HIGH-PRECISION SCOPE", font=("Helvetica", 11, "bold"), fg=FG_CYAN, bg=BG_CARD).pack(side="left")

        stats_lbl = tk.Label(hdr, text="Ref: 0.00 | Act: 0.00 | Err: 0.00", font=("Consolas", 10, "bold"), fg=FG_WHITE, bg=BG_CARD)
        stats_lbl.pack(side="right")

        canvas_frame = tk.Frame(win, bg=BG_DARK, pady=10, padx=10)
        canvas_frame.pack(fill="both", expand=True)

        expanded_canvas = tk.Canvas(canvas_frame, bg=BG_PLOT_CANVAS, highlightthickness=1, highlightbackground=COLOR_PLOT_AXIS)
        expanded_canvas.pack(fill="both", expand=True)

        legend_frame = tk.Frame(win, bg=BG_CARD, pady=4, padx=10)
        legend_frame.pack(fill="x")
        tk.Label(legend_frame, text="─── Reference Setpoint", fg=COLOR_PLOT_REF, bg=BG_CARD, font=("Consolas", 9, "bold")).pack(side="left", padx=10)
        tk.Label(legend_frame, text="─── Actual Telemetry", fg=COLOR_PLOT_ACT, bg=BG_CARD, font=("Consolas", 9, "bold")).pack(side="left", padx=10)

        self.active_popups[axis] = {
            "window": win,
            "canvas": expanded_canvas,
            "stats_lbl": stats_lbl
        }

        def _on_close():
            if axis in self.active_popups:
                del self.active_popups[axis]
            win.destroy()

        win.protocol("WM_DELETE_WINDOW", _on_close)

    def update_view(self):
        """Consulta la telemetría del ControlReceiver y actualiza buffers."""
        rx = self.control_rx or getattr(ControlReceiver, "_active_instance", None)
        if rx is None:
            return

        if hasattr(rx, "get_control_data"):
            pos_ref, pos_act = rx.get_control_data()
            self.update_position(pos_ref, pos_act)

    def update_position(self, pos_ref: list, pos_act: list):
        """Añade vectores de telemetría y redibuja los canvas inline y popups."""
        if len(pos_ref) >= 3 and len(pos_act) >= 3:
            for idx, axis in enumerate(["X", "Y", "Z"]):
                ref_val = float(pos_ref[idx])
                act_val = float(pos_act[idx])

                self.history[axis]["ref"].append(ref_val)
                self.history[axis]["act"].append(act_val)

                # Renderizar inline canvas
                self._render_axis_canvas(self.canvases[axis], axis)

                # Renderizar popup abierto
                if axis in self.active_popups:
                    popup = self.active_popups[axis]
                    if popup["window"].winfo_exists():
                        err = act_val - ref_val
                        popup["stats_lbl"].config(
                            text=f"Ref: {ref_val:+.2f} | Act: {act_val:+.2f} | Err: {err:+.2f}"
                        )
                        self._render_axis_canvas(popup["canvas"], axis, is_expanded=True)

    def _render_axis_canvas(self, canvas: tk.Canvas, axis: str, is_expanded: bool = False):
        """Motor de graficado estilo MATLAB con grilla oscura y curvas de trayectoria."""
        canvas.delete("all")

        w = canvas.winfo_width()
        h = canvas.winfo_height()

        if w < 20 or h < 20:
            w = 680 if is_expanded else 300
            h = 300 if is_expanded else 65

        ref_data = list(self.history[axis]["ref"])
        act_data = list(self.history[axis]["act"])

        min_val = min(min(ref_data), min(act_data)) - 0.2
        max_val = max(max(ref_data), max(act_data)) + 0.2
        if max_val - min_val < 0.4:
            max_val += 0.2
            min_val -= 0.2

        range_val = max_val - min_val
        dx = w / (self.max_history - 1)

        # Grilla de fondo
        y_divs = 8 if is_expanded else 4
        x_divs = 12 if is_expanded else 6

        for i in range(1, y_divs):
            gy = (h / y_divs) * i
            canvas.create_line(0, gy, w, gy, fill=COLOR_PLOT_GRID, dash=(2, 4))

        for i in range(1, x_divs):
            gx = (w / x_divs) * i
            canvas.create_line(gx, 0, gx, h, fill=COLOR_PLOT_GRID, dash=(2, 4))

        # Línea de referencia Cero
        if min_val <= 0.0 <= max_val:
            zero_y = h - ((0.0 - min_val) / range_val * h)
            canvas.create_line(0, zero_y, w, zero_y, fill=COLOR_PLOT_ZERO, width=1.5)

        # Trayectoria Setpoint (Línea punteada roja)
        ref_pts = []
        for i, val in enumerate(ref_data):
            x = i * dx
            y = h - ((val - min_val) / range_val * h)
            ref_pts.extend([x, y])

        if len(ref_pts) >= 4:
            canvas.create_line(ref_pts, fill=COLOR_PLOT_REF, width=2.0 if is_expanded else 1.5, dash=(4, 3))

        # Telemetría Real (Línea sólida verde)
        act_pts = []
        for i, val in enumerate(act_data):
            x = i * dx
            y = h - ((val - min_val) / range_val * h)
            act_pts.extend([x, y])

        if len(act_pts) >= 4:
            canvas.create_line(act_pts, fill=COLOR_PLOT_ACT, width=2.5 if is_expanded else 2.0, smooth=True, splinesteps=12)

        # Etiquetas de escala en ejes
        font_size = 9 if is_expanded else 7
        canvas.create_text(8, 8, text=f"Max: {max_val:.2f}", fill=COLOR_PLOT_TEXT, font=("Consolas", font_size, "bold"), anchor="nw")
        canvas.create_text(8, h - 8, text=f"Min: {min_val:.2f}", fill=COLOR_PLOT_TEXT, font=("Consolas", font_size, "bold"), anchor="sw")