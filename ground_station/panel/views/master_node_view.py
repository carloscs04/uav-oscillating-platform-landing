# ground_station/panel/views/master_node_view.py

import tkinter as tk
from ground_station.utils.colors import BG_CARD, FG_CYAN, FG_WHITE, ACCENT_GREEN, ACCENT_RED, ACCENT_YELLOW

class MasterNodeView(tk.LabelFrame):
    """System Master Node badge and stage display."""
    
    def __init__(self, parent, controller, **kwargs):
        super().__init__(parent, text=" System Master Node ", font=("Helvetica", 10, "bold"),
                         fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=4, **kwargs)
        self.controller = controller

        self.lbl_stage = tk.Label(self, text=f"Active Stage : {self.controller.current_stage}", font=("Consolas", 9, "bold"), fg=FG_WHITE, bg=BG_CARD, anchor="w")
        self.lbl_stage.pack(fill="x", padx=8)
        
        self.lbl_master_state = tk.Label(self, text="Master State : [ DISCONNECTED ]", font=("Consolas", 9), fg=ACCENT_RED, bg=BG_CARD, anchor="w")
        self.lbl_master_state.pack(fill="x", padx=8)

    def update_view(self):
        self.lbl_stage.config(text=f"Active Stage : {self.controller.current_stage}")
        m_state = self.controller.master_state
        m_color = ACCENT_GREEN if m_state in ["ACTIVE", "OK"] else (ACCENT_YELLOW if m_state in ["POOR", "CONNECTING"] else ACCENT_RED)
        self.lbl_master_state.config(text=f"Master State : [ {m_state} ]", fg=m_color)