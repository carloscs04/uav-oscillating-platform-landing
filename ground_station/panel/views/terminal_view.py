# ground_station/panel/views/terminal_view.py

import tkinter as tk

BG_CARD = "#102a45"
FG_CYAN = "#00f0ff"
FG_WHITE = "#ffffff"

class EmbeddedTerminalView(tk.LabelFrame):
    def __init__(self, parent, on_execute_command=None):
        super().__init__(parent, text=" Interactive GCS Console ",
                         font=("Helvetica", 10, "bold"), fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove")
        self.on_execute_command = on_execute_command

        # Terminal Output Box
        self.log_area = tk.Text(self, bg="#040b14", fg=FG_CYAN, font=("Consolas", 9),
                                height=7, wrap="word", bd=0, highlightthickness=0)
        self.log_area.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Command Input Entry Row
        input_frame = tk.Frame(self, bg=BG_CARD)
        input_frame.pack(fill="x", padx=5, pady=(0, 5))

        prompt_lbl = tk.Label(input_frame, text="GCS> ", font=("Consolas", 10, "bold"), fg=FG_WHITE, bg=BG_CARD)
        prompt_lbl.pack(side="left")

        self.cmd_entry = tk.Entry(input_frame, bg="#040b14", fg=FG_WHITE, insertbackground=FG_WHITE,
                                  font=("Consolas", 10), bd=1, relief="solid")
        self.cmd_entry.pack(side="left", fill="x", expand=True, padx=5)
        self.cmd_entry.bind("<Return>", self._send_command)

        btn_send = tk.Button(input_frame, text="Send", bg="#00897b", fg="white",
                             font=("Helvetica", 8, "bold"), command=self._send_command)
        btn_send.pack(side="right")

        self.log_message("[SYSTEM] Ground Station CLI Console Ready. Type 'help' for available commands.")

    def log_message(self, msg: str):
        self.log_area.insert("end", f"{msg}\n")
        self.log_area.see("end")

    def _send_command(self, event=None):
        cmd = self.cmd_entry.get().strip()
        if not cmd:
            return
        
        self.log_message(f"GCS> {cmd}")
        self.cmd_entry.delete(0, "end")

        if self.on_execute_command:
            self.on_execute_command(cmd)