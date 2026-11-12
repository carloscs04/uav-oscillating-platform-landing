# File: ground_station/panel/views/cmd_master.py

import time
import tkinter as tk
from ground_station.utils.colors import (
    BG_CARD, BG_DARK, FG_CYAN, FG_WHITE,
    ACCENT_GREEN, ACCENT_RED, ACCENT_YELLOW
)
from ground_station.receivers.nodes.master_rx import MasterReceiver


class MasterTerminalView(tk.LabelFrame):
    """Remote Console Terminal Viewer & CLI Command Sender exclusively for Master Node."""

    def __init__(self, parent, master_rx=None, **kwargs):
        super().__init__(
            parent, text=" Master Node Stream & Command Console ", font=("Helvetica", 10, "bold"),
            fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=4, padx=6, **kwargs
        )
        self.master_rx = master_rx if hasattr(master_rx, "get_all_packets") else None
        self._last_printed_seq: int = 0
        self._current_boot_id = None
        self._is_stream_online: bool = False
        self._cmd_history = []
        self._history_idx = -1

        # 1. Top FSM & Active Subsystems Header Bar
        hdr_frame = tk.Frame(self, bg=BG_CARD)
        hdr_frame.pack(fill="x", pady=(0, 3))

        self.lbl_fsm = tk.Label(
            hdr_frame, text="FSM: [OFFLINE] | DISARMED",
            font=("Consolas", 8, "bold"), fg=ACCENT_RED, bg=BG_CARD
        )
        self.lbl_fsm.pack(side="left")

        self.lbl_nodes = tk.Label(
            hdr_frame, text="NODES: 0/5",
            font=("Consolas", 8, "bold"), fg=FG_WHITE, bg=BG_CARD
        )
        self.lbl_nodes.pack(side="right")

        # 2. Terminal Log Output Display
        txt_frame = tk.Frame(self, bg=BG_DARK)
        txt_frame.pack(fill="both", expand=True, pady=(0, 4))

        self.txt = tk.Text(txt_frame, bg="#040b14", fg=ACCENT_GREEN, font=("Consolas", 9), bd=0, height=8, wrap="word")
        scrollbar = tk.Scrollbar(txt_frame, command=self.txt.yview)
        self.txt.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.txt.pack(side="left", fill="both", expand=True)

        self.txt.tag_config("INFO", foreground=ACCENT_GREEN)
        self.txt.tag_config("OK", foreground=ACCENT_GREEN)
        self.txt.tag_config("SYSTEM", foreground=FG_CYAN)
        self.txt.tag_config("CMD", foreground="#ffffff")
        self.txt.tag_config("STATUS", foreground="#80deea")
        self.txt.tag_config("WARN", foreground=ACCENT_YELLOW)
        self.txt.tag_config("BLOCKED", foreground=ACCENT_RED)
        self.txt.tag_config("ALERT", foreground=ACCENT_RED)
        self.txt.tag_config("ERROR", foreground=ACCENT_RED)

        self.txt.insert("end", "=== MASTER NODE CLI STREAM & CONSOLE ===\n", "SYSTEM")

        # 3. Command Line Input Box
        cmd_frame = tk.Frame(self, bg=BG_CARD)
        cmd_frame.pack(fill="x")

        tk.Label(cmd_frame, text="CMD>", font=("Consolas", 9, "bold"), fg=FG_WHITE, bg=BG_CARD).pack(side="left", padx=(0, 4))
        self.entry_cmd = tk.Entry(cmd_frame, bg=BG_DARK, fg=FG_WHITE, font=("Consolas", 9), insertbackground="white", bd=1)
        self.entry_cmd.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.entry_cmd.bind("<Return>", lambda e: self.send_cmd())
        self.entry_cmd.bind("<Up>", self._history_up)
        self.entry_cmd.bind("<Down>", self._history_down)

        btn_send = tk.Button(
            cmd_frame, text="Execute", bg="#3949ab", fg="white",
            font=("Helvetica", 8, "bold"), bd=0, relief="flat", padx=8, pady=2, cursor="hand2",
            command=self.send_cmd
        )
        btn_send.pack(side="right")

        self._resolve_receiver()
        self._start_loop()

    def _resolve_receiver(self):
        """Resolves or auto-starts the MasterReceiver singleton on port 5008."""
        rx = self.master_rx or getattr(MasterReceiver, "_active_instance", None)
        if rx is None:
            rx = MasterReceiver(port=5008)
            rx.start()
        elif not rx.running and not rx.is_alive():
            rx.start()
        self.master_rx = rx
        return rx

    def append_log(self, msg: str, tag: str = "INFO"):
        if not hasattr(self, "txt") or not self.txt.winfo_exists():
            return

        msg_str = str(msg)
        if msg_str.startswith("[") and len(msg_str) > 10 and msg_str[3] == ":":
            formatted = msg_str if msg_str.endswith("\n") else f"{msg_str}\n"
        else:
            timestamp = time.strftime("%H:%M:%S")
            formatted = f"[{timestamp}] {msg_str}\n" if not msg_str.endswith("\n") else f"[{timestamp}] {msg_str}"

        try:
            self.txt.insert("end", formatted, tag)
            self.txt.see("end")

            line_count = int(self.txt.index("end-1c").split(".")[0])
            if line_count > 250:
                self.txt.delete("1.0", "20.0")
        except Exception:
            pass

    def send_cmd(self):
        """Sends a CLI command directly to RPi MasterCommandReceiver."""
        cmd_text = self.entry_cmd.get().strip()
        if not cmd_text:
            return

        self.entry_cmd.delete(0, "end")
        self._cmd_history.append(cmd_text)
        self._history_idx = len(self._cmd_history)

        if cmd_text.lower() == "clear":
            self.txt.delete("1.0", "end")
            self.txt.insert("end", "=== MASTER NODE CLI STREAM & CONSOLE ===\n", "SYSTEM")
            return

        rx = self._resolve_receiver()
        if not rx.send_terminal_command(cmd_text):
            self.append_log("[ERROR] RPi IP unknown or link disconnected. Command not sent.", "ERROR")

    def _start_loop(self):
        self.update_view()
        self.after(120, self._start_loop)

    def update_view(self):
        rx = self._resolve_receiver()
        if not rx:
            return

        last_seen = getattr(rx, "last_seen", 0.0)
        is_alive = (last_seen > 0.0) and ((time.time() - last_seen) <= 2.5)

        # Detect RPi process stop / link timeout
        if not is_alive:
            if self._is_stream_online:
                self._is_stream_online = False
                self._last_printed_seq = 0
                self._current_boot_id = None
                self.lbl_fsm.config(text="FSM: [OFFLINE] | DISARMED", fg=ACCENT_RED)
                self.lbl_nodes.config(text="NODES: 0/5")
                self.append_log("[ALERT] MasterNode stream lost (RPi process stopped or offline).", "ALERT")
            return

        all_packets = rx.get_all_packets()
        if not all_packets:
            return

        # 1. Detect RPi process startup / reconnection
        cli_stream = all_packets.get("cli_stream", {})
        if isinstance(cli_stream, dict):
            boot_id = cli_stream.get("boot_id")
            latest_seq = cli_stream.get("latest_seq", 0)

            # Reset sequence counter if RPi restarted with a new boot_id or lower seq
            if (boot_id is not None and boot_id != self._current_boot_id) or (latest_seq < self._last_printed_seq):
                if self._current_boot_id is not None:
                    self.append_log("──────────────────────────────────────────────────────────", "SYSTEM")
                    self.append_log("[SYSTEM] New MasterNode session detected (RPi restarted).", "SYSTEM")
                self._current_boot_id = boot_id
                self._last_printed_seq = 0

            if not self._is_stream_online:
                self._is_stream_online = True
                if boot_id is None:
                    self.append_log("[SYSTEM] MasterNode stream CONNECTED.", "SYSTEM")

            # Stream new sequenced console lines
            lines = cli_stream.get("lines", [])
            for entry in lines:
                seq = entry.get("seq", 0)
                text = entry.get("text", "")
                if seq > self._last_printed_seq and text:
                    self._last_printed_seq = seq
                    tag = self._detect_log_tag(text)
                    self.append_log(text, tag)

        # 2. Update Top FSM Status Bar
        m_status = all_packets.get("master_status", {})
        if isinstance(m_status, dict) and m_status:
            fsm_state = m_status.get("system_state", "UNKNOWN")
            armed = m_status.get("armed", False)
            arm_str = "ARMED" if armed else "DISARMED"
            color = ACCENT_RED if fsm_state == "EMERGENCY_ABORT" else (
                ACCENT_YELLOW if armed else ACCENT_GREEN
            )
            self.lbl_fsm.config(text=f"FSM: [{fsm_state}] | {arm_str}", fg=color)

        status_pkt = all_packets.get("status", {})
        if isinstance(status_pkt, dict) and "active_nodes" in status_pkt:
            act = status_pkt.get("active_nodes", 0)
            tot = status_pkt.get("total_nodes", 5)
            self.lbl_nodes.config(text=f"NODES: {act}/{tot}")

    def _detect_log_tag(self, line: str) -> str:
        for tag in ("BLOCKED", "ALERT", "ERROR", "WARN", "SYSTEM", "STATUS", "CMD", "OK"):
            if f"[{tag}]" in line:
                return tag
        return "INFO"

    def _history_up(self, _event):
        if self._cmd_history and self._history_idx > 0:
            self._history_idx -= 1
            self.entry_cmd.delete(0, "end")
            self.entry_cmd.insert(0, self._cmd_history[self._history_idx])

    def _history_down(self, _event):
        if self._cmd_history and self._history_idx < len(self._cmd_history) - 1:
            self._history_idx += 1
            self.entry_cmd.delete(0, "end")
            self.entry_cmd.insert(0, self._cmd_history[self._history_idx])
        else:
            self._history_idx = len(self._cmd_history)
            self.entry_cmd.delete(0, "end")


# Alias for export to master_panel.py
ConsoleView = MasterTerminalView