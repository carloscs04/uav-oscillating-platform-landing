# ground_station/panel/views/console_view.py

import tkinter as tk
from ground_station.utils.colors import BG_CARD, FG_CYAN, FG_WHITE

class ConsoleView(tk.LabelFrame):
    """Stream logger & Command CLI Entry Bar."""
    
    def __init__(self, parent, controller, **kwargs):
        super().__init__(parent, text=" Master Node Stream & Command Console ", font=("Helvetica", 10, "bold"),
                         fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", **kwargs)
        self.controller = controller

        self.console_log = tk.Text(self, bg="#040b14", fg=FG_CYAN, font=("Consolas", 9), height=6, wrap="word", bd=0, highlightthickness=0)
        self.console_log.pack(fill="both", expand=True, padx=6, pady=5)

        input_frame = tk.Frame(self, bg=BG_CARD)
        input_frame.pack(fill="x", padx=6, pady=(0, 6))

        tk.Label(input_frame, text="GCS> ", font=("Consolas", 10, "bold"), fg=FG_WHITE, bg=BG_CARD).pack(side="left")

        self.cmd_entry = tk.Entry(input_frame, bg="#040b14", fg=FG_WHITE, insertbackground=FG_WHITE, font=("Consolas", 10), bd=1, relief="solid")
        self.cmd_entry.pack(side="left", fill="x", expand=True, padx=5)
        self.cmd_entry.bind("<Return>", self._on_send_command)

        btn_send = tk.Button(input_frame, text="Send", bg="#00897b", fg="white", font=("Helvetica", 9, "bold"), command=self._on_send_command)
        btn_send.pack(side="right")

    def append_log(self, text: str):
        self.console_log.insert("end", f"{text}\n")
        self.console_log.see("end")

    def _on_send_command(self, event=None):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self.controller.send_cli_command(cmd)
            self.cmd_entry.delete(0, "end")