# File: ground_station/panel/views/actions_view.py

import time
import tkinter as tk
from ground_station.utils.colors import BG_CARD, FG_CYAN
from ground_station.receivers.nodes.master_rx import MasterReceiver
from ground_station.receivers.nodes.vision_rx import VisionReceiver
from ground_station.transmiters.main_command_tx import MainCommandTransmitter


class ActionsView(tk.LabelFrame):
    """GUIDED landing mission. START enables the stages, STOP holds and disables them. These
    buttons never change the flight mode: the pilot enters and leaves GUIDED with channel 5."""

    COLOR_IDLE = "#37474f"
    COLOR_WAITING = "#f9a825"
    COLOR_ACTIVE = "#2e7d32"
    COLOR_START = "#00695c"
    COLOR_STOP = "#ef6c00"
    LEVELS = ("align", "descend", "land")

    def __init__(self, parent, controller=None, **kwargs):
        super().__init__(
            parent, text=" Landing Controls (GUIDED) ", font=("Helvetica", 10, "bold"),
            fg=FG_CYAN, bg=BG_CARD, bd=2, relief="groove", pady=8, **kwargs
        )
        self.controller = controller
        self.cmd_tx = getattr(MainCommandTransmitter, "_active_instance", None) or MainCommandTransmitter()

        self._live_stage: str = ""
        self._mission: bool = False
        self._last_hb_tx: float = 0.0

        btn_grid = tk.Frame(self, bg=BG_CARD)
        btn_grid.pack(anchor="w", padx=10)

        b_style = {"font": ("Helvetica", 9, "bold"), "bd": 0, "padx": 10, "pady": 6, "cursor": "hand2"}

        self.btn_mission = tk.Button(
            btn_grid, text="▶ START MISSION", bg=self.COLOR_START, fg="white",
            command=self.cmd_mission, **b_style
        )
        self.btn_mission.grid(row=0, column=0, padx=(0, 5))

        self.stage_buttons = {}
        for col, (stage, label) in enumerate((
            ("align", "🎯 ALIGN"),
            ("descend", "⬇ APPROACH 35 cm"),
            ("land", "🛬 TOUCHDOWN"),
        ), start=1):
            btn = tk.Button(btn_grid, text=label, bg=self.COLOR_IDLE, fg="white", state="disabled",
                            disabledforeground="#90a4ae", command=lambda s=stage: self.cmd_stage(s), **b_style)
            btn.grid(row=0, column=col, padx=5)
            self.stage_buttons[stage] = btn

        self.btn_abort = tk.Button(
            btn_grid, text="⚠ ABORT", bg="#c62828", fg="white",
            command=self.cmd_abort, **b_style
        )
        self.btn_abort.grid(row=0, column=4, padx=(5, 0))

        self._sync_ui_loop()

    def _send_silent_heartbeat(self):
        """Sends a 1 Hz silent ping to RPi MasterNode so the 10s disconnect failsafe knows GCS is alive."""
        now = time.time()
        if now - self._last_hb_tx < 1.0:
            return
        self._last_hb_tx = now

        rx = getattr(MasterReceiver, "_active_instance", None)
        if rx and hasattr(rx, "send_terminal_command"):
            rx.send_terminal_command("gcs_hb")
        else:
            self.cmd_tx.send_command("gcs_hb")

    def _sync_ui_loop(self):
        """Polls live telemetry and lights up the stage the Pi really has selected."""
        self._send_silent_heartbeat()

        rx = getattr(MasterReceiver, "_active_instance", None)
        armed = False
        fsm_state = "OFFLINE"
        online = False

        if rx:
            last_seen = getattr(rx, "last_seen", 0.0)
            if last_seen > 0.0 and (time.time() - last_seen) <= 2.5:
                online = True
                packets = rx.get_all_packets() if hasattr(rx, "get_all_packets") else {}
                m_status = packets.get("master_status", {}) if isinstance(packets, dict) else {}
                if isinstance(m_status, dict) and m_status:
                    armed = bool(m_status.get("armed", False))
                    fsm_state = str(m_status.get("system_state", "READY")).upper()

        if self.controller:
            self.controller.is_armed = armed
            self.controller.in_flight = (fsm_state == "MISSION_ACTIVE")

        vis_rx = getattr(VisionReceiver, "_active_instance", None)
        shadow = (vis_rx.get_packet("landing_shadow", {}) or {}) if (vis_rx and online) else {}
        state, _, stage = str(shadow.get("mode", "SHADOW")).partition("-")
        self._live_stage = stage
        self._mission = bool(shadow.get("mission"))
        self.btn_mission.config(
            text="■ STOP MISSION" if self._mission else "▶ START MISSION",
            bg=self.COLOR_STOP if self._mission else self.COLOR_START,
        )
        level = self.LEVELS.index(stage) if self._mission and stage in self.LEVELS else -1
        for i, name in enumerate(self.LEVELS):      # APPROACH and TOUCHDOWN include the stages before
            color = self.COLOR_IDLE
            if i <= level:
                color = self.COLOR_ACTIVE if state == "ACTIVE" else self.COLOR_WAITING
            self.stage_buttons[name].config(bg=color, state="normal" if self._mission else "disabled")

        self.after(200, self._sync_ui_loop)

    def _dispatch_to_master(self, cli_command: str, log_msg: str) -> bool:
        rx = getattr(MasterReceiver, "_active_instance", None)
        if rx and hasattr(rx, "send_terminal_command"):
            sent = rx.send_terminal_command(cli_command)
        else:
            sent = self.cmd_tx.send_command(cli_command)

        if self.controller and hasattr(self.controller, "_emit_log"):
            if sent:
                self.controller._emit_log(log_msg)
            else:
                self.controller._emit_log("[ERROR] RPi MasterNode OFFLINE. Command not sent.")
        return sent

    def cmd_mission(self):
        if self._mission:
            self._dispatch_to_master("landshadow stop", "[TX] STOP MISSION: holding, stages disabled.")
        else:
            self._dispatch_to_master("landshadow start", "[TX] START MISSION: holding, choose a stage.")

    def cmd_stage(self, name: str):
        """On = run up to this stage. Off = fall back to the previous one (ALIGN off = hold)."""
        i = self.LEVELS.index(name)
        current = self.LEVELS.index(self._live_stage) if self._live_stage in self.LEVELS else -1
        target = self.LEVELS[i] if current < i else (self.LEVELS[i - 1] if i else "hold")
        self._dispatch_to_master(f"landshadow arm {target}", f"[TX] Stage: {target}.")

    def cmd_abort(self):
        self._dispatch_to_master("abort", "[EMERGENCY] 'abort' sent to MasterNode!")
