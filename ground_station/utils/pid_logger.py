# file: ground_station/utils/pid_logger.py
import csv
import math
import os
import threading
import time
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional, TextIO, Tuple


class PidTuningLogger:
    """Compact control log for tuning the landing law, written on the ground station.

    Only while ARMED in GUIDED (the part where the law flies), one row per NEW vision frame
    (repeated packets are skipped), few columns, rounded. Next to it a short _summary.txt with
    per-phase errors, swings and losses, so a flight can be reviewed without the full vision log.
    -> experiments/vision/pid/<YYYY-MM-DD>/pid_<HH-MM-SS>.csv + pid_<HH-MM-SS>_summary.txt"""

    GUIDED = 4
    CLOSE_GRACE_S = 1.5
    HEADERS = ("t", "det", "id", "h", "e_fwd", "e_right", "err", "phase",
               "v_fwd", "v_right", "v_down", "sent_fwd", "sent_right", "sent_down",
               "roll", "pitch", "note")

    def __init__(self, base_dir: Optional[str] = None,
                 mode_source: Optional[Callable[[], Optional[Tuple[bool, int]]]] = None):
        if base_dir is None:
            repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
            base_dir = os.path.join(repo_root, "experiments", "vision", "pid")
        self.base_dir = base_dir
        if mode_source is None:
            from ground_station.utils.vision_logger import VisionExperimentLogger
            mode_source = VisionExperimentLogger._mavlink_mode
        self.mode_source = mode_source
        self._lock = threading.Lock()
        self._file: Optional[TextIO] = None
        self._writer = None
        self._path = ""
        self._t0 = 0.0
        self._left_since: Optional[float] = None
        self._last_key = None
        self._rows: List[dict] = []

    def process_packets(self, packets: Dict[str, Any], fresh=None, now: Optional[float] = None):
        now = time.time() if now is None else now
        try:
            state = self.mode_source()
        except Exception:
            state = None
        allowed = bool(state) and state[0] and state[1] == self.GUIDED
        with self._lock:
            if allowed:
                self._left_since = None
                if self._file is None:
                    self._open_locked(now)
            elif self._file is not None:
                self._left_since = self._left_since or now
                if now - self._left_since >= self.CLOSE_GRACE_S:
                    self._close_locked()
            if allowed and self._file is not None and (fresh is None or "pixel_target" in fresh):
                self._write_locked(packets, now)

    def _open_locked(self, now: float):
        day_dir = os.path.join(self.base_dir, datetime.now().strftime("%Y-%m-%d"))
        os.makedirs(day_dir, exist_ok=True)
        name, n = datetime.now().strftime("pid_%H-%M-%S"), 2
        self._path = os.path.join(day_dir, f"{name}.csv")
        while os.path.exists(self._path):
            self._path, n = os.path.join(day_dir, f"{name}_{n}.csv"), n + 1
        self._file = open(self._path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADERS)
        self._t0, self._last_key, self._rows = now, None, []
        print(f"[PID_LOGGER] Recording -> {self._path}")

    @staticmethod
    def _r(v, nd=3):
        if v is None or v == "":
            return ""
        try:
            return round(float(v), nd)
        except (TypeError, ValueError):
            return ""

    def _write_locked(self, packets: Dict[str, Any], now: float):
        pt = packets.get("pixel_target") or {}
        ls = packets.get("landing_shadow") or {}
        det = int(bool(pt.get("detected")))
        src = str(pt.get("source") or "")
        mid = src.split("#")[-1] if "#" in src else ""
        row = {
            "t": round(now - self._t0, 2), "det": det, "id": mid, "h": self._r(pt.get("h_m")),
            "e_fwd": self._r(pt.get("pad_fwd_m")), "e_right": self._r(pt.get("pad_right_m")),
            "err": self._r(ls.get("err_m")), "phase": ls.get("phase", ""),
            "v_fwd": self._r(ls.get("v_fwd")), "v_right": self._r(ls.get("v_right")),
            "v_down": self._r(ls.get("v_down")),
            "sent_fwd": self._r(ls.get("sent_v_fwd")), "sent_right": self._r(ls.get("sent_v_right")),
            "sent_down": self._r(ls.get("sent_v_down")),
            "roll": self._r(pt.get("roll_deg"), 1), "pitch": self._r(pt.get("pitch_deg"), 1),
            "note": ls.get("note", "") or "",
        }
        key = tuple(v for k, v in row.items() if k != "t")
        if key == self._last_key:          # same frame resent: skip
            return
        self._last_key = key
        self._writer.writerow([row[k] for k in self.HEADERS])
        self._rows.append(row)

    def _close_locked(self):
        if self._file is None:
            return
        self._file.close()
        self._file, self._writer = None, None
        summary = self._path[:-4] + "_summary.txt"
        try:
            with open(summary, "w", encoding="utf-8") as f:
                f.write(self.summarize(self._rows))
        except Exception as e:
            print(f"[PID_LOGGER] summary failed: {e}")
        print(f"[PID_LOGGER] Saved {len(self._rows)} rows -> {self._path}")

    @staticmethod
    def summarize(rows: List[dict]) -> str:
        if not rows:
            return "no rows\n"
        out = [f"duration {rows[-1]['t']:.1f} s, {len(rows)} rows"]
        lost = sum(1 for a, b in zip(rows, rows[1:]) if a["det"] and not b["det"])
        out.append(f"marker lost {lost} times")
        out.append("phase      time_s  h_range_m      err_mean_cm  err_max_cm  swings/s  v_cmd_max_cm/s")
        phases = []
        for r in rows:
            if r["phase"] and (not phases or phases[-1] != r["phase"]):
                phases.append(r["phase"])
        for ph in dict.fromkeys(phases):
            sel = [r for r in rows if r["phase"] == ph]
            dur = sum(b["t"] - a["t"] for a, b in zip(rows, rows[1:]) if a["phase"] == ph)
            errs = [r["err"] for r in sel if r["err"] != ""]
            hs = [r["h"] for r in sel if r["h"] != ""]
            vs = [math.hypot(r["v_fwd"] or 0, r["v_right"] or 0) for r in sel if r["v_fwd"] != ""]
            swings = 0
            for k in ("e_fwd", "e_right"):          # sign changes of an error larger than 1 cm
                s_prev = 0
                for r in sel:
                    v = r[k]
                    if v == "" or abs(v) < 0.01:
                        continue
                    s = 1 if v > 0 else -1
                    swings += s_prev and s != s_prev
                    s_prev = s
            hr = f"{min(hs):.2f}-{max(hs):.2f}" if hs else "-"
            em = f"{100 * sum(errs) / len(errs):.1f}" if errs else "-"
            ex = f"{100 * max(errs):.1f}" if errs else "-"
            out.append(f"{ph:<10} {dur:6.1f}  {hr:<13}  {em:>11}  {ex:>10}  {swings / max(dur, 0.1):8.2f}"
                       f"  {100 * max(vs) if vs else 0:14.1f}")
        # vertical: commanded hold/climb vs what the lidar saw (sinking while asked to climb)
        sink = [(a, b) for a, b in zip(rows, rows[1:])
                if a["sent_down"] != "" and a["sent_down"] < -0.02 and a["h"] != "" and b["h"] != ""]
        if sink:
            dh = sum(b["h"] - a["h"] for a, b in sink)
            dt = sum(b["t"] - a["t"] for a, b in sink)
            out.append(f"asked to climb for {dt:.1f} s: lidar changed {100 * dh:+.1f} cm")
        notes: Dict[str, int] = {}
        for r in rows:
            n = r["note"].split(":")[0] if r["note"] else ""
            if n:
                notes[n] = notes.get(n, 0) + 1
        out.append("notes: " + ", ".join(f"{k} x{v}" for k, v in sorted(notes.items(), key=lambda x: -x[1])))
        last = next((r for r in reversed(rows) if r["det"]), None)
        if last:
            out.append(f"last seen: h {last['h']} m, err {last['err']} m, phase {last['phase']}")
        out.append(f"end phase {rows[-1]['phase']}, note '{rows[-1]['note']}'")
        return "\n".join(out) + "\n"

    def close(self):
        with self._lock:
            self._close_locked()
