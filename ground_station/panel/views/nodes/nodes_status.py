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
    """Sub-Nodes Health Matrix & Single-Packet Terminal Inspector."""

    def __init__(self, parent, logic_panel, pi_link=None, **kwargs):
        super().__init__(
            parent, text=" Sub-Nodes Health Matrix ", font=("Helvetica", 10, "bold"),
            fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=4, **kwargs
        )
        self.logic_panel = logic_panel
        self.pi_link = pi_link

        self.main_receiver = MainReceiver()
        self.main_receiver.start_all()

        self.status_widgets = {}
        self.toggle_buttons = {}
        self.node_enabled_states = {}
        self.active_inspectors = {}

        initial_nodes = self.main_receiver.get_all_node_statuses()
        for name, info in initial_nodes.items():
            self.node_enabled_states[name] = True

            row = tk.Frame(self, bg=BG_CARD)
            row.pack(fill="x", padx=8, pady=2)

            tk.Label(row, text=f"• {name}", font=("Consolas", 9), fg=FG_BLUE, bg=BG_CARD, anchor="w").pack(side="left")

            right_subframe = tk.Frame(row, bg=BG_CARD)
            right_subframe.pack(side="right")

            lbl_stat = tk.Label(right_subframe, text=f"[{info['status']}]", font=("Consolas", 9, "bold"), fg=ACCENT_RED, bg=BG_CARD)
            lbl_stat.pack(side="left", padx=(0, 8))
            self.status_widgets[name] = lbl_stat

            btn_inspect = tk.Button(
                right_subframe, text="Inspect Data", bg="#3949ab", fg="white",
                font=("Helvetica", 8, "bold"), bd=0, relief="flat", padx=6, pady=2, cursor="hand2",
                command=lambda n=name: self.open_node_inspector(n)
            )
            btn_inspect.pack(side="left", padx=(0, 6))

            btn_toggle = tk.Frame(right_subframe, bg=ACCENT_GREEN, width=14, height=14, cursor="hand2")
            btn_toggle.pack_propagate(False)
            btn_toggle.pack(side="left", padx=(4, 0))

            btn_toggle.bind("<Button-1>", lambda e, n=name, f=btn_toggle: self.toggle_node_status(n, f))
            self.toggle_buttons[name] = btn_toggle

    def toggle_node_status(self, node_name: str, frame_widget: tk.Frame):
        is_enabled = not self.node_enabled_states.get(node_name, True)
        self.node_enabled_states[node_name] = is_enabled

        frame_widget.config(bg=ACCENT_GREEN if is_enabled else "#FFFFFF")

        # Update local receiver thread enable state
        if hasattr(self.main_receiver, "control_rx") and self.main_receiver.control_rx:
            if node_name == "3-Axis Control Node":
                self.main_receiver.control_rx.enabled = is_enabled
        if hasattr(self.main_receiver, "mavlink_rx") and self.main_receiver.mavlink_rx:
            if node_name == "PX4-MAVLink Node":
                self.main_receiver.mavlink_rx.enabled = is_enabled
        if hasattr(self.main_receiver, "camera_rx") and self.main_receiver.camera_rx:
            if node_name == "Aruco-Perception Node":
                self.main_receiver.camera_rx.enabled = is_enabled
        if hasattr(self.main_receiver, "distance_rx") and self.main_receiver.distance_rx:
            if node_name == "Distance-Sensor Node":
                self.main_receiver.distance_rx.enabled = is_enabled

        active_link = self.pi_link or getattr(PiLink, "_active_instance", None)
        if active_link:
            cmd = "START_STREAM" if is_enabled else "STOP_STREAM"
            active_link.send_command(cmd, {"node": node_name})

    def open_node_inspector(self, node_name: str):
        """Opens clean single-packet terminal inspector popup."""
        if node_name in self.active_inspectors:
            info = self.active_inspectors[node_name]
            if info["win"].winfo_exists():
                info["win"].lift()
                info["win"].focus_set()
                return

        win = tk.Toplevel(self.winfo_toplevel())
        win.title(f"Interactive Packet Terminal - {node_name}")
        win.geometry("640x420")
        win.configure(bg=BG_DARK)

        # Header Info Bar
        hdr_frame = tk.Frame(win, bg=BG_CARD, pady=6, padx=10)
        hdr_frame.pack(fill="x")

        tk.Label(hdr_frame, text=f"LIVE FEED: {node_name.upper()}", font=("Helvetica", 10, "bold"), fg=FG_CYAN, bg=BG_CARD).pack(side="left")
        hdr_status_lbl = tk.Label(hdr_frame, text="HEALTH: OK | STATE: INITIALIZING", font=("Consolas", 9, "bold"), fg=ACCENT_YELLOW, bg=BG_CARD)
        hdr_status_lbl.pack(side="right")

        # Dynamic Packet Tab Bar
        tabs_frame = tk.Frame(win, bg=BG_DARK, pady=4, padx=8)
        tabs_frame.pack(fill="x")

        selected_packet_var = tk.StringVar(value="")

        # Terminal Text Box
        txt_frame = tk.Frame(win, bg=BG_DARK, padx=8, pady=4)
        txt_frame.pack(fill="both", expand=True)

        txt = tk.Text(txt_frame, bg="#040b14", fg=ACCENT_GREEN, font=("Consolas", 9), bd=0, wrap="none")
        scrollbar = tk.Scrollbar(txt_frame, command=txt.yview)
        txt.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        txt.pack(side="left", fill="both", expand=True)

        txt.insert("end", f"=== {node_name} PACKET TERMINAL ===\n")
        txt.insert("end", "Select a packet tab above to inspect telemetry in real-time.\n" + "─" * 60 + "\n\n")

        self.active_inspectors[node_name] = {
            "win": win,
            "txt": txt,
            "hdr_lbl": hdr_status_lbl,
            "tabs_frame": tabs_frame,
            "active_packet_var": selected_packet_var,
            "rendered_tabs": set(),
            "last_pkt_str": None
        }

        def _on_close():
            if node_name in self.active_inspectors:
                del self.active_inspectors[node_name]
            win.destroy()

        win.protocol("WM_DELETE_WINDOW", _on_close)

    def update_matrix(self):
        """Refreshes status matrix indicators and updates active terminal windows smoothly."""
        node_states = self.main_receiver.get_all_node_statuses()

        for name, info in node_states.items():
            st = info.get("status", "DISCONNECTED")
            is_enabled = self.node_enabled_states.get(name, True)

            # 1. Update Status Badge Colors in Main Matrix
            if name in self.status_widgets:
                display_text = f"[{st}]" if is_enabled else "[DISABLED]"
                c = ACCENT_GREEN if (st in ["OK", "STREAMING"] and is_enabled) else "#888888" if not is_enabled else ACCENT_RED
                self.status_widgets[name].config(text=display_text, fg=c)

            # 2. Update Active Terminal Inspectors
            if name in self.active_inspectors:
                inspector = self.active_inspectors[name]
                if not inspector["win"].winfo_exists():
                    del self.active_inspectors[name]
                    continue

                txt = inspector["txt"]
                hdr_lbl = inspector["hdr_lbl"]
                tabs_frame = inspector["tabs_frame"]
                telemetry = info.get("telemetry", {})

                # Update Header Node Health Status
                health_data = telemetry.get("node_health", {})
                h_status = health_data.get("status", "HEALTHY" if is_enabled else "OFFLINE")
                hdr_lbl.config(
                    text=f"HEALTH: [{h_status}] | STATE: [{st if is_enabled else 'DISABLED'}] | PORT: {info.get('port', 5007)}",
                    fg=ACCENT_GREEN if h_status == "HEALTHY" else ACCENT_RED
                )

                # Dynamically create Packet Tabs (excluding internal 'node_health')
                for pkt_key in telemetry.keys():
                    if pkt_key not in inspector["rendered_tabs"] and pkt_key != "node_health":
                        inspector["rendered_tabs"].add(pkt_key)

                        # FIX: Bind BOTH n=name AND k=pkt_key locally to fix closure scope issue
                        btn_pkt = tk.Button(
                            tabs_frame, text=f"[ {pkt_key} ]", bg="#2c387e", fg="white",
                            font=("Helvetica", 8, "bold"), bd=0, relief="flat", padx=8, pady=2, cursor="hand2",
                            command=lambda k=pkt_key, n=name: self._select_packet_tab(n, k)
                        )
                        btn_pkt.pack(side="left", padx=2)

                        # Auto-select first discovered packet tab
                        if not inspector["active_packet_var"].get():
                            self._select_packet_tab(name, pkt_key)

                # Append log line ONLY when current tab's content changes
                active_pkt = inspector["active_packet_var"].get()
                current_val = telemetry.get(active_pkt) if active_pkt else None

                if is_enabled and current_val is not None:
                    current_str = json.dumps(current_val)
                    last_str = inspector.get("last_pkt_str")

                    if current_str != last_str:
                        inspector["last_pkt_str"] = current_str
                        timestamp = time.strftime("%H:%M:%S")
                        txt.insert("end", f"[{timestamp}] [{active_pkt}]: {current_str}\n")
                        txt.see("end")

                        # Prevent console text buffer overflow
                        line_count = int(txt.index("end-1c").split(".")[0])
                        if line_count > 200:
                            txt.delete("1.0", "2.0")

    def _select_packet_tab(self, node_name: str, pkt_name: str):
        """Switches terminal view to the selected packet tab and notifies RPi."""
        if node_name in self.active_inspectors:
            inspector = self.active_inspectors[node_name]
            inspector["active_packet_var"].set(pkt_name)
            inspector["last_pkt_str"] = None  # Force immediate refresh on tab switch

            txt = inspector["txt"]
            txt.insert("end", f"\n>>> SWITCHED TO PACKET TAB: [{pkt_name}]\n")
            txt.see("end")

            active_link = self.pi_link or getattr(PiLink, "_active_instance", None)
            if active_link:
                active_link.send_command("SELECT_PACKET", {"node": node_name, "packet": pkt_name})