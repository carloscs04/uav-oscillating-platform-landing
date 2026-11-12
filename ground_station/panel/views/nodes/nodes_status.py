# File: ground_station/panel/views/nodes/nodes_status.py

import json
import time
import tkinter as tk
from ground_station.utils.colors import (
    BG_CARD, BG_DARK, FG_CYAN, FG_BLUE, FG_WHITE,
    ACCENT_GREEN, ACCENT_RED, ACCENT_YELLOW
)
from ground_station.receivers.main_receiver import MainReceiver
from ground_station.comms.pi_bridge import PiLink, LinkState
from ground_station.receivers.nodes.mavlink_rx import MavlinkReceiver
from ground_station.receivers.nodes.vision_rx import VisionReceiver


class NodeHealthView(tk.LabelFrame):
    """Sub-Nodes Health Matrix & Isolated Multi-Tab Packet Terminal Inspector."""

    KNOWN_NODE_PACKETS = {
        "MAVLink Node": ["velocity", "attitude", "sys_status", "heartbeat"],
        "Control Node": ["control_state", "pid_output"],
        "Vision-ArUco Node": ["aruco_pose", "status"],
        "Camera-Aruco Node": ["aruco_pose", "status"],
        "Distance-Sensor Node": ["distance_sensor"]
    }

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

            lbl_stat = tk.Label(right_subframe, text="[OFFLINE]", font=("Consolas", 9, "bold"), fg=ACCENT_RED, bg=BG_CARD)
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
        """Toggles local reception and sends START/STOP stream commands to the RPi."""
        is_enabled = not self.node_enabled_states.get(node_name, True)
        self.node_enabled_states[node_name] = is_enabled

        frame_widget.config(bg=ACCENT_GREEN if is_enabled else "#FFFFFF")

        rx_obj = self._get_receiver_for_node(node_name)
        if rx_obj and hasattr(rx_obj, "enabled"):
            rx_obj.enabled = is_enabled

        active_link = self.pi_link or getattr(PiLink, "_active_instance", None)
        if active_link:
            cmd = "START_STREAM" if is_enabled else "STOP_STREAM"
            active_link.send_command(cmd, {"node": node_name})

    def open_node_inspector(self, node_name: str):
        """Opens an isolated multi-tab packet terminal window for a specific node."""
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

        hdr_frame = tk.Frame(win, bg=BG_CARD, pady=6, padx=10)
        hdr_frame.pack(fill="x")

        tk.Label(
            hdr_frame, text=f"LIVE FEED: {node_name.upper()}",
            font=("Helvetica", 10, "bold"), fg=FG_CYAN, bg=BG_CARD
        ).pack(side="left")

        hdr_status_lbl = tk.Label(
            hdr_frame, text="INITIALIZING...",
            font=("Consolas", 9, "bold"), fg=ACCENT_YELLOW, bg=BG_CARD
        )
        hdr_status_lbl.pack(side="right")

        tabs_frame = tk.Frame(win, bg=BG_DARK, pady=4, padx=8)
        tabs_frame.pack(fill="x")

        selected_packet_var = tk.StringVar(value="")

        txt_frame = tk.Frame(win, bg=BG_DARK, padx=8, pady=4)
        txt_frame.pack(fill="both", expand=True)

        txt = tk.Text(txt_frame, bg="#040b14", fg=ACCENT_GREEN, font=("Consolas", 9), bd=0, wrap="none")
        scrollbar = tk.Scrollbar(txt_frame, command=txt.yview)
        txt.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        txt.pack(side="left", fill="both", expand=True)

        self.active_inspectors[node_name] = {
            "win": win,
            "txt": txt,
            "hdr_lbl": hdr_status_lbl,
            "tabs_frame": tabs_frame,
            "active_packet_var": selected_packet_var,
            "rendered_tabs": set(),
            "tab_buttons": {},
            "tab_logs": {},
            "last_pkt_strs": {}
        }

        default_pkts = self.KNOWN_NODE_PACKETS.get(node_name, [])
        for pkt_key in default_pkts:
            self._create_tab_button(node_name, pkt_key)

        def _on_close():
            if node_name in self.active_inspectors:
                del self.active_inspectors[node_name]
            win.destroy()

        win.protocol("WM_DELETE_WINDOW", _on_close)

    def _create_tab_button(self, node_name: str, pkt_key: str):
        """Creates a dedicated tab button and isolated log buffer for each packet key."""
        if node_name not in self.active_inspectors:
            return
        inspector = self.active_inspectors[node_name]
        if pkt_key in inspector["rendered_tabs"] or pkt_key == "node_health":
            return

        inspector["rendered_tabs"].add(pkt_key)
        inspector["tab_logs"].setdefault(pkt_key, [])
        inspector["last_pkt_strs"].setdefault(pkt_key, None)

        tabs_frame = inspector["tabs_frame"]

        btn_pkt = tk.Button(
            tabs_frame, text=f"[ {pkt_key} ]", bg="#1c2541", fg="#b0bec5",
            font=("Helvetica", 8, "bold"), bd=0, relief="flat", padx=8, pady=3, cursor="hand2",
            command=lambda k=pkt_key, n=node_name: self._select_packet_tab(n, k)
        )
        btn_pkt.pack(side="left", padx=2)
        inspector["tab_buttons"][pkt_key] = btn_pkt

        if not inspector["active_packet_var"].get():
            self._select_packet_tab(node_name, pkt_key)

    def _format_health_header(self, health_info: dict) -> str:
        """Dynamically builds terminal header metrics string from node_health payload."""
        if not health_info:
            return ""

        parts = []
        for k, v in health_info.items():
            if k == "status":
                continue

            if k == "rate_hz":
                parts.append(f"rate_hz: {v}Hz")
            elif k == "voltage_v":
                parts.append(f"voltage_v: {v}V")
            elif k == "battery_pct":
                val_str = f"{v}%" if isinstance(v, (int, float)) and v >= 0 else f"{v}"
                parts.append(f"battery_pct: {val_str}")
            else:
                parts.append(f"{k}: {v}")

        return " | ".join(parts)

    def _get_receiver_for_node(self, node_name: str):
        """Resolves sub-node receiver with active Singleton fallback."""
        if node_name in ["MAVLink Node", "PX4-MAVLink Node"]:
            active_inst = getattr(MavlinkReceiver, "_active_instance", None)
            if active_inst:
                return active_inst

        if node_name in ["Vision-ArUco Node", "Camera-Aruco Node", "Aruco-Perception Node", "Vision Node"]:
            active_vis = getattr(VisionReceiver, "_active_instance", None)
            if active_vis:
                return active_vis

        if not self.main_receiver:
            return None

        name_map = {
            "MAVLink Node": ["mavlink_rx", "mavlink_receiver", "mavlink"],
            "PX4-MAVLink Node": ["mavlink_rx", "mavlink_receiver", "mavlink"],
            "Control Node": ["control_rx", "control_receiver", "control"],
            "3-Axis Control Node": ["control_rx", "control_receiver", "control"],
            "Vision-ArUco Node": ["vision_rx", "vision_receiver", "perception_rx", "vision"],
            "Camera-Aruco Node": ["vision_rx", "vision_receiver", "perception_rx", "vision"],
            "Aruco-Perception Node": ["vision_rx", "vision_receiver", "perception_rx", "vision"],
            "Distance-Sensor Node": ["distance_rx", "distance_receiver", "distance"]
        }

        aliases = name_map.get(node_name, [])
        for attr in aliases:
            obj = getattr(self.main_receiver, attr, None)
            if obj is not None:
                return obj
        return None

    def update_matrix(self):
        """Refreshes status matrix badges and updates the active packet view."""
        node_states = self.main_receiver.get_all_node_statuses()

        active_link = self.pi_link or getattr(PiLink, "_active_instance", None)
        is_master_connected = False
        if active_link and hasattr(active_link, "get_health"):
            is_master_connected = (active_link.get_health().get("state") == LinkState.CONNECTED)

        for name, info in node_states.items():
            is_enabled = self.node_enabled_states.get(name, True)
            rx_obj = self._get_receiver_for_node(name)

            if not is_master_connected:
                h_status = "OFFLINE"
                health_info = {}
            else:
                health_info = rx_obj.get_health_info() if (rx_obj and hasattr(rx_obj, "get_health_info")) else {}
                h_status = health_info.get("status", "OFFLINE")

            # 1. Update Matrix Status Badge
            if name in self.status_widgets:
                if not is_enabled:
                    display_text = "[DISABLED]"
                    c = "#888888"
                elif h_status in ["HEALTHY", "POOR"]:
                    display_text = f"[{h_status}] [OK]"
                    c = ACCENT_GREEN if h_status == "HEALTHY" else ACCENT_YELLOW
                else:
                    display_text = "[OFFLINE]"
                    c = ACCENT_RED

                self.status_widgets[name].config(text=display_text, fg=c)

            # 2. Update Terminal Header and Active Tab Buffer
            if name in self.active_inspectors:
                inspector = self.active_inspectors[name]
                if not inspector["win"].winfo_exists():
                    del self.active_inspectors[name]
                    continue

                txt = inspector["txt"]
                hdr_lbl = inspector["hdr_lbl"]
                telemetry = info.get("telemetry", {})

                if h_status in ["HEALTHY", "POOR"]:
                    hdr_text = self._format_health_header(health_info)
                    hdr_color = ACCENT_GREEN if h_status == "HEALTHY" else ACCENT_YELLOW
                else:
                    hdr_text = "HEALTH: [OFFLINE]"
                    hdr_color = ACCENT_RED

                hdr_lbl.config(text=hdr_text, fg=hdr_color)

                # Dynamically register tabs for any new keys discovered in incoming JSON
                for pkt_key in telemetry.keys():
                    self._create_tab_button(name, pkt_key)

                if rx_obj and hasattr(rx_obj, "packets") and isinstance(rx_obj.packets, dict):
                    for pkt_key in rx_obj.packets.keys():
                        self._create_tab_button(name, pkt_key)

                active_pkt = inspector["active_packet_var"].get()
                if not active_pkt:
                    continue

                current_val = None
                if rx_obj:
                    if hasattr(rx_obj, "get_packet"):
                        current_val = rx_obj.get_packet(active_pkt)
                    elif hasattr(rx_obj, "packets") and isinstance(rx_obj.packets, dict):
                        current_val = rx_obj.packets.get(active_pkt)

                if current_val is None:
                    current_val = telemetry.get(active_pkt)

                if is_enabled and current_val is not None:
                    current_str = json.dumps(current_val)
                    last_str = inspector["last_pkt_strs"].get(active_pkt)

                    if current_str != last_str:
                        inspector["last_pkt_strs"][active_pkt] = current_str
                        timestamp = time.strftime("%H:%M:%S")
                        log_line = f"[{timestamp}] [{active_pkt}]: {current_str}\n"

                        # Append to dedicated tab history buffer (max 200 entries)
                        tab_history = inspector["tab_logs"].setdefault(active_pkt, [])
                        tab_history.append(log_line)
                        if len(tab_history) > 200:
                            tab_history.pop(0)

                        # Append to visible terminal text widget
                        txt.insert("end", log_line)
                        txt.see("end")

                        line_count = int(txt.index("end-1c").split(".")[0])
                        if line_count > 205:
                            txt.delete("4.0", "5.0")

    def _select_packet_tab(self, node_name: str, pkt_name: str):
        """Replaces the terminal screen with the isolated view for the selected packet tab."""
        if node_name not in self.active_inspectors:
            return

        inspector = self.active_inspectors[node_name]
        inspector["active_packet_var"].set(pkt_name)
        inspector["last_pkt_strs"][pkt_name] = None

        # Highlight active tab button and dim inactive tabs
        for key, btn in inspector["tab_buttons"].items():
            if key == pkt_name:
                btn.config(bg="#0288d1", fg="white")
            else:
                btn.config(bg="#1c2541", fg="#b0bec5")

        # Wipe current view and render isolated packet channel view
        txt = inspector["txt"]
        txt.delete("1.0", "end")
        txt.insert("end", f"=== {node_name.upper()} | PACKET CHANNEL: [{pkt_name}] ===\n")
        txt.insert("end", "─" * 62 + "\n\n")

        history = inspector["tab_logs"].get(pkt_name, [])
        if history:
            txt.insert("end", "".join(history))
        txt.see("end")

        active_link = self.pi_link or getattr(PiLink, "_active_instance", None)
        if active_link:
            cmd_payload = {
                "node": node_name,
                "packet": pkt_name,
                "filter": pkt_name
            }
            active_link.send_command("SELECT_PACKET", cmd_payload)
            active_link.send_command("SET_FILTER", cmd_payload)