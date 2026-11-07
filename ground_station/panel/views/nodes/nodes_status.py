# ground_station/panel/views/nodes/nodes_status.py

import json
import time
import tkinter as tk
from ground_station.utils.colors import (
    BG_CARD, BG_DARK, FG_CYAN, FG_BLUE, FG_WHITE,
    ACCENT_GREEN, ACCENT_RED, ACCENT_YELLOW
)
from ground_station.receivers.main_receiver import MainReceiver
from ground_station.comms.pi_bridge import PiLink


class NodeHealthView(tk.LabelFrame):
    """Sub-Nodes Health Matrix view. Instantiates MainReceiver and provides live data terminal inspectors."""

    def __init__(self, parent, logic_panel, pi_link=None, **kwargs):
        super().__init__(
            parent, text=" Sub-Nodes Health Matrix ", font=("Helvetica", 10, "bold"),
            fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=4, **kwargs
        )
        self.logic_panel = logic_panel
        self.pi_link = pi_link

        # Instantiate MainReceiver and link PiLink if available
        self.main_receiver = MainReceiver()
        if hasattr(self.main_receiver, "control_rx") and hasattr(self.main_receiver.control_rx, "pi_link"):
            self.main_receiver.control_rx.pi_link = self.pi_link

        self.main_receiver.start_all()

        self.status_widgets = {}
        self.toggle_buttons = {}      # Guardará referencias a los cuadros de estado
        self.node_enabled_states = {} # Controla si el nodo está habilitado {node_name: bool}
        self.active_inspectors = {}   # Tracks open popup windows

        initial_nodes = self.main_receiver.get_all_node_statuses()
        for name, info in initial_nodes.items():
            self.node_enabled_states[name] = True  # Inicia habilitado por defecto

            row = tk.Frame(self, bg=BG_CARD)
            row.pack(fill="x", padx=8, pady=2)

            tk.Label(row, text=f"• {name}", font=("Consolas", 9), fg=FG_BLUE, bg=BG_CARD, anchor="w").pack(side="left")

            right_subframe = tk.Frame(row, bg=BG_CARD)
            right_subframe.pack(side="right")

            lbl_stat = tk.Label(right_subframe, text=f"[{info['status']}]", font=("Consolas", 9, "bold"), fg=ACCENT_RED, bg=BG_CARD)
            lbl_stat.pack(side="left", padx=(0, 8))
            self.status_widgets[name] = lbl_stat

            # 1. Botón Inspect Data
            btn_inspect = tk.Button(
                right_subframe, text="Inspect Data", bg="#3949ab", fg="white",
                font=("Helvetica", 8, "bold"), bd=0, relief="flat", padx=6, pady=2, cursor="hand2",
                command=lambda n=name: self.open_node_inspector(n)
            )
            btn_inspect.pack(side="left", padx=(0, 6))

            # 2. Cuadro Indicador (Cuadrado de 14x14 píxeles, Verde/Blanco)
            btn_toggle = tk.Frame(
                right_subframe, bg=ACCENT_GREEN, width=14, height=14, cursor="hand2"
            )
            btn_toggle.pack_propagate(False)
            btn_toggle.pack(side="left", padx=(4, 0))

            btn_toggle.bind("<Button-1>", lambda e, n=name, f=btn_toggle: self.toggle_node_status(n, f))
            self.toggle_buttons[name] = btn_toggle

    def toggle_node_status(self, node_name: str, frame_widget: tk.Frame):
        """Cambia el estado de activación del nodo (Verde = ON / Blanco = OFF) y notifica a la RPi."""
        is_enabled = not self.node_enabled_states.get(node_name, True)
        self.node_enabled_states[node_name] = is_enabled

        # 1. Actualizar apariencia visual
        if is_enabled:
            frame_widget.config(bg=ACCENT_GREEN)
        else:
            frame_widget.config(bg="#FFFFFF")

        # 2. Notificar al receptor local en la Laptop (ControlReceiver)
        if hasattr(self.main_receiver, "control_rx") and self.main_receiver.control_rx:
            if node_name == "3-Axis Control Node":
                self.main_receiver.control_rx.enabled = is_enabled

        # 3. Notificar a la Raspberry Pi usando la instancia activa de PiLink
        active_link = self.pi_link or getattr(PiLink, "_active_instance", None)
        if active_link:
            cmd = "START_STREAM" if is_enabled else "STOP_STREAM"
            active_link.send_command(cmd, {"node": node_name})

    def open_node_inspector(self, node_name: str):
        """Opens a live real-time terminal monitor window for the selected node."""
        if node_name in self.active_inspectors:
            info = self.active_inspectors[node_name]
            if info["win"].winfo_exists():
                info["win"].lift()
                info["win"].focus_set()
                return

        win = tk.Toplevel(self.winfo_toplevel())
        win.title(f"Node Terminal Inspector - {node_name}")
        win.geometry("620x380")
        win.configure(bg=BG_DARK)

        # Top Header Bar
        hdr_frame = tk.Frame(win, bg=BG_CARD, pady=6, padx=10)
        hdr_frame.pack(fill="x")

        tk.Label(hdr_frame, text=f"LIVE TERMINAL FEED: {node_name.upper()}", font=("Helvetica", 10, "bold"), fg=FG_CYAN, bg=BG_CARD).pack(side="left")
        hdr_status_lbl = tk.Label(hdr_frame, text="STATUS: INITIALIZING", font=("Consolas", 9, "bold"), fg=ACCENT_YELLOW, bg=BG_CARD)
        hdr_status_lbl.pack(side="right")

        # Terminal Text Widget
        txt_frame = tk.Frame(win, bg=BG_DARK, padx=8, pady=8)
        txt_frame.pack(fill="both", expand=True)

        txt = tk.Text(txt_frame, bg="#040b14", fg=ACCENT_GREEN, font=("Consolas", 9), bd=0, wrap="none")
        scrollbar = tk.Scrollbar(txt_frame, command=txt.yview)
        txt.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side="right", fill="y")
        txt.pack(side="left", fill="both", expand=True)

        txt.insert("end", f"=== {node_name} LIVE TELEMETRY TERMINAL ===\n")
        txt.insert("end", f"Console initialized at: {time.strftime('%H:%M:%S')}\n")
        txt.insert("end", "─" * 60 + "\n\n")

        self.active_inspectors[node_name] = {
            "win": win,
            "txt": txt,
            "hdr_lbl": hdr_status_lbl,
            "last_payload": None
        }

        def _on_close():
            if node_name in self.active_inspectors:
                del self.active_inspectors[node_name]
            win.destroy()

        win.protocol("WM_DELETE_WINDOW", _on_close)

    def update_matrix(self):
        """Refreshes status badge colors and streams live terminal logs to active inspector windows."""
        node_states = self.main_receiver.get_all_node_statuses()

        for name, info in node_states.items():
            st = info.get("status", "DISCONNECTED")
            is_enabled = self.node_enabled_states.get(name, True)

            # 1. Update Status Badge Colors in Main Matrix
            if name in self.status_widgets:
                if not is_enabled:
                    c = "#888888"
                    display_text = "[DISABLED]"
                elif st in ["OK", "ONLINE", "STREAMING", "HANDSHAKE_ACK_SENT", "CONNECTED"]:
                    c = ACCENT_GREEN
                    display_text = f"[{st}]"
                elif st in ["POOR", "STANDBY", "WAIT_BEACON", "WAITING_FOR_BEACON", "WAITING_FOR_LINK"]:
                    c = ACCENT_YELLOW
                    display_text = f"[{st}]"
                else:
                    c = ACCENT_RED
                    display_text = f"[{st}]"

                self.status_widgets[name].config(text=display_text, fg=c)

            # 2. Update Active Inspector Windows with Live Terminal Data
            if name in self.active_inspectors:
                inspector = self.active_inspectors[name]
                
                if not inspector["win"].winfo_exists():
                    del self.active_inspectors[name]
                    continue

                txt = inspector.get("txt")
                hdr_lbl = inspector.get("hdr_lbl") or inspector.get("hdr_status_lbl")

                # Update Header Badge Status
                if hdr_lbl:
                    hdr_color = ACCENT_GREEN if (st in ["OK", "ONLINE", "STREAMING", "ACTIVE_STREAM", "CONNECTED"] and is_enabled) else ACCENT_YELLOW if st in ["STANDBY", "WAIT_BEACON"] else ACCENT_RED
                    status_str = f"STATE: [{st if is_enabled else 'DISABLED'}] | PORT: {info.get('port', 5007)} | RATE: {info.get('rate', 20)}Hz"
                    hdr_lbl.config(text=status_str, fg=hdr_color)

                telemetry = info.get("telemetry", {})
                
                # Detect fresh payload
                last_payload = inspector.get("last_payload")
                if is_enabled and telemetry and telemetry != last_payload:
                    inspector["last_payload"] = dict(telemetry) if isinstance(telemetry, dict) else telemetry
                    timestamp = time.strftime("%H:%M:%S")

                    if name == "3-Axis Control Node":
                        ref = telemetry.get("ref", [0.0, 0.0, 0.0])
                        act = telemetry.get("act", [0.0, 0.0, 0.0])
                        rx_state = info.get("status", "STREAMING")
                        line = f"[{timestamp}] State: {rx_state} | Ref: {ref} | Act: {act}\n"
                    elif name == "Distance-Sensor Node":
                        dist = telemetry.get("distance", 0.0)
                        line = f"[{timestamp}] Rangefinder: {dist:.2f} m\n"
                    else:
                        line = f"[{timestamp}] Telemetry RX: {json.dumps(telemetry)}\n"

                    if txt:
                        txt.insert("end", line)
                        txt.see("end")

                        line_count = int(txt.index("end-1c").split(".")[0])
                        if line_count > 200:
                            txt.delete("1.0", "2.0")