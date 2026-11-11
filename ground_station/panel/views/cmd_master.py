# File: ground_station/panel/views/cmd_master.py

import json
import time
import tkinter as tk
from ground_station.utils.colors import (
    BG_CARD, BG_DARK, FG_CYAN, FG_WHITE, ACCENT_GREEN, ACCENT_RED
)
from ground_station.receivers.nodes.master_rx import MasterReceiver


class MasterTerminalView(tk.LabelFrame):
    """Remote Console Terminal Viewer & CLI Command Sender exclusively for Master Node."""

    def __init__(self, parent, master_rx=None, **kwargs):
        super().__init__(
            parent, text=" Master Node Stream & Command Console ", font=("Helvetica", 10, "bold"),
            fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=4, padx=6, **kwargs
        )
        self.master_rx = master_rx

        # 1. Terminal Log Output Display
        txt_frame = tk.Frame(self, bg=BG_DARK)
        txt_frame.pack(fill="both", expand=True, pady=(0, 4))

        self.txt = tk.Text(txt_frame, bg="#040b14", fg=ACCENT_GREEN, font=("Consolas", 9), bd=0, height=8, wrap="word")
        scrollbar = tk.Scrollbar(txt_frame, command=self.txt.yview)
        self.txt.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.txt.pack(side="left", fill="both", expand=True)

        self.txt.insert("end", "=== MASTER NODE CLI STREAM & CONSOLE ===\n")

        # 2. Command Line Input Box
        cmd_frame = tk.Frame(self, bg=BG_CARD)
        cmd_frame.pack(fill="x")

        tk.Label(cmd_frame, text="CMD>", font=("Consolas", 9, "bold"), fg=FG_WHITE, bg=BG_CARD).pack(side="left", padx=(0, 4))
        self.entry_cmd = tk.Entry(cmd_frame, bg=BG_DARK, fg=FG_WHITE, font=("Consolas", 9), insertbackground="white", bd=1)
        self.entry_cmd.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.entry_cmd.bind("<Return>", lambda e: self.send_cmd())

        btn_send = tk.Button(
            cmd_frame, text="Execute", bg="#3949ab", fg="white",
            font=("Helvetica", 8, "bold"), bd=0, relief="flat", padx=8, pady=2, cursor="hand2",
            command=self.send_cmd
        )
        btn_send.pack(side="right")

        self._last_packets_hash = ""
        self._start_loop()

    def append_log(self, msg: str):
        """Callback invocado por logic_panel para insertar mensajes del sistema en la consola."""
        if not hasattr(self, "txt") or not self.txt.winfo_exists():
            return

        timestamp = time.strftime("%H:%M:%S")
        formatted = f"[{timestamp}] {msg}\n" if not str(msg).endswith("\n") else f"[{timestamp}] {msg}"

        try:
            self.txt.insert("end", formatted)
            self.txt.see("end")

            line_count = int(self.txt.index("end-1c").split(".")[0])
            if line_count > 200:
                self.txt.delete("1.0", "20.0")
        except Exception:
            pass

    def send_cmd(self):
        """Envía un comando CLI directamente a la Raspberry Pi a través de MasterReceiver."""
        cmd_text = self.entry_cmd.get().strip()
        if not cmd_text:
            return

        self.append_log(f"CMD> {cmd_text}")
        self.entry_cmd.delete(0, "end")

        rx = self.master_rx or getattr(MasterReceiver, "_active_instance", None)
        if rx and hasattr(rx, "send_terminal_command"):
            rx.send_terminal_command(cmd_text)

    def _start_loop(self):
        self.update_view()
        self.after(100, self._start_loop)

    def update_view(self):
        """Lee de forma continua todos los paquetes entrantes del MasterReceiver."""
        rx = self.master_rx or getattr(MasterReceiver, "_active_instance", None)
        if not rx:
            return

        all_packets = rx.get_all_packets() if hasattr(rx, "get_all_packets") else getattr(rx, "packets", {})
        if not all_packets:
            return

        current_hash = json.dumps(all_packets)
        if current_hash != self._last_packets_hash:
            self._last_packets_hash = current_hash
            timestamp = time.strftime("%H:%M:%S")

            for pkt_type, pkt_content in all_packets.items():
                if isinstance(pkt_content, (dict, list)):
                    content_str = json.dumps(pkt_content)
                else:
                    content_str = str(pkt_content)

                self.txt.insert("end", f"[{timestamp}] [{pkt_type}]: {content_str}\n")

            self.txt.see("end")

            line_count = int(self.txt.index("end-1c").split(".")[0])
            if line_count > 200:
                self.txt.delete("1.0", "20.0")


# Alias para exportación hacia master_panel.py
ConsoleView = MasterTerminalView