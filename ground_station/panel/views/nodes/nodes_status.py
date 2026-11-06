# ground_station/panel/views/nodes/nodes_status.py

import tkinter as tk
from ground_station.utils.colors import BG_CARD, BG_DARK, FG_CYAN, FG_BLUE, ACCENT_GREEN, ACCENT_RED, ACCENT_YELLOW
from ground_station.receivers.main_receiver import MainReceiver


class NodeHealthView(tk.LabelFrame):
    """Sub-Nodes Health Matrix view. Instantiates and manages MainReceiver internally."""

    def __init__(self, parent, logic_panel, **kwargs):
        super().__init__(parent, text=" Sub-Nodes Health Matrix ", font=("Helvetica", 10, "bold"),
                         fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=4, **kwargs)
        self.logic_panel = logic_panel

        # Instantiates and starts MainReceiver directly inside this view
        self.main_receiver = MainReceiver()
        self.main_receiver.start_all()

        self.status_widgets = {}

        initial_nodes = self.main_receiver.get_all_node_statuses()
        for name, info in initial_nodes.items():
            row = tk.Frame(self, bg=BG_CARD)
            row.pack(fill="x", padx=8, pady=2)

            tk.Label(row, text=f"• {name}", font=("Consolas", 9), fg=FG_BLUE, bg=BG_CARD, anchor="w").pack(side="left")

            right_subframe = tk.Frame(row, bg=BG_CARD)
            right_subframe.pack(side="right")

            lbl_stat = tk.Label(right_subframe, text=f"[{info['status']}]", font=("Consolas", 9, "bold"), fg=ACCENT_RED, bg=BG_CARD)
            lbl_stat.pack(side="left", padx=(0, 8))
            self.status_widgets[name] = lbl_stat

            btn_inspect = tk.Button(right_subframe, text="Inspect Data", bg="#3949ab", fg="white", font=("Helvetica", 8, "bold"),
                                    command=lambda n=name: self.open_node_inspector(n))
            btn_inspect.pack(side="right")

    def open_node_inspector(self, node_name: str):
        win = tk.Toplevel(self.winfo_toplevel())
        win.title(f"Node Inspector - {node_name}")
        win.geometry("550x320")
        win.configure(bg=BG_DARK)

        info = self.main_receiver.get_all_node_statuses().get(node_name, {})
        hdr = tk.Label(win, text=f"LIVE TELEMETRY STREAM: {node_name}", font=("Helvetica", 11, "bold"), fg=FG_CYAN, bg=BG_CARD, pady=6)
        hdr.pack(fill="x")

        txt = tk.Text(win, bg="#040b14", fg=ACCENT_GREEN, font=("Consolas", 9), bd=0)
        txt.pack(fill="both", expand=True, padx=10, pady=10)
        txt.insert("end", f"Connecting to {node_name} on UDP Port {info.get('port')}...\n")
        txt.insert("end", f"Data Stream Rate: {info.get('rate')}\n")
        txt.insert("end", f"Current Node Health Status: [{info.get('status')}]\n")
        txt.insert("end", f"Payload: {info.get('telemetry')}\n")

    def update_matrix(self):
        node_states = self.main_receiver.get_all_node_statuses()
        for name, info in node_states.items():
            if name in self.status_widgets:
                st = info["status"]
                c = ACCENT_GREEN if st in ["OK", "ONLINE"] else (ACCENT_YELLOW if st in ["POOR", "STANDBY"] else ACCENT_RED)
                self.status_widgets[name].config(text=f"[{st}]", fg=c)