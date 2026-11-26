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
    (repeated packets are skipped). Next to it a short _summary.txt with per-phase errors,
    swings, losses and how the Pixhawk followed the velocity commands (gain, lag).
    -> experiments/vision/pid/<YYYY-MM-DD>/pid_<HH-MM-SS>.csv + pid_<HH-MM-SS>_summary.txt

    Columns: e_* = touchdown point seen by the camera (m, + = ahead/right), rate_* = its rate
    (the D input, m/s), v_* = law command, sent_* = what the sender sent, fc_* = velocity the
    Pixhawk measured (body frame, from LOCAL_POSITION_NED rotated by yaw), fc_h = EKF height."""

    GUIDED = 4
    CLOSE_GRACE_S = 1.5
    FC_MAX_AGE_S = 1.0
    HEADERS = ("t", "det", "id", "h", "e_fwd", "e_right", "err", "rate_fwd", "rate_right",
               "phase", "gains", "committed", "v_fwd", "v_right", "v_down",
               "sent_fwd", "sent_right", "sent_down", "fc_fwd", "fc_right", "fc_down", "fc_h",
               "flow_q", "roll", "pitch", "stage", "reason", "note")
    _VOLATILE = {"t", "fc_fwd", "fc_right", "fc_down", "fc_h", "flow_q"}

    def __init__(self, base_dir: Optional[str] = None,
                 mode_source: Optional[Callable[[], Optional[Tuple[bool, int]]]] = None,
                 fc_source: Optional[Callable[[], Dict[str, Any]]] = None):
        if base_dir is None:
            repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
            base_dir = os.path.join(repo_root, "experiments", "vision", "pid")
        self.base_dir = base_dir
        if mode_source is None:
            from ground_station.utils.vision_logger import VisionExperimentLogger
            mode_source = VisionExperimentLogger._mavlink_mode
        self.mode_source = mode_source
        self.fc_source = fc_source or self._mavlink_fc
        self._lock = threading.Lock()
        self._file: Optional[TextIO] = None
        self._writer = None
        self._path = ""
        self._t0 = 0.0
        self._left_since: Optional[float] = None
        self._last_key = None
        self._rows: List[dict] = []

    @classmethod
    def _mavlink_fc(cls) -> Dict[str, Any]:
        """Pixhawk velocity in the body frame (fwd, right, down), EKF height, flow quality."""
        from ground_station.receivers.nodes.mavlink_rx import MavlinkReceiver
        rx = getattr(MavlinkReceiver, "_active_instance", None)
        if rx is None or time.time() - getattr(rx, "last_seen", 0.0) > cls.FC_MAX_AGE_S:
            return {}
        lp = rx.get_packet("local_position") or {}
        att = rx.get_packet("attitude") or {}
        flow = rx.get_packet("optical_flow") or {}
        if "vx" not in lp or "yaw" not in att:
            return {}
        yaw = math.radians(float(att["yaw"]))
        vn, ve = float(lp["vx"]), float(lp["vy"])
        return {"fc_fwd": vn * math.cos(yaw) + ve * math.sin(yaw),
                "fc_right": -vn * math.sin(yaw) + ve * math.cos(yaw),
                "fc_down": float(lp.get("vz", 0.0)), "fc_h": -float(lp.get("z", 0.0)),
                "flow_q": flow.get("quality", "")}

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
        try:
            fc = self.fc_source() or {}
        except Exception:
            fc = {}
        src = str(pt.get("source") or "")
        r = self._r
        row = {
            "t": round(now - self._t0, 2), "det": int(bool(pt.get("detected"))),
            "id": src.split("#")[-1] if "#" in src else "", "h": r(pt.get("h_m")),
            "e_fwd": r(pt.get("pad_fwd_m")), "e_right": r(pt.get("pad_right_m")),
            "err": r(ls.get("err_m")), "rate_fwd": r(ls.get("rate_fwd")),
            "rate_right": r(ls.get("rate_right")), "phase": ls.get("phase", ""),
            "gains": ls.get("gains", ""), "committed": ls.get("committed", ""),
            "v_fwd": r(ls.get("v_fwd")), "v_right": r(ls.get("v_right")), "v_down": r(ls.get("v_down")),
            "sent_fwd": r(ls.get("sent_v_fwd")), "sent_right": r(ls.get("sent_v_right")),
            "sent_down": r(ls.get("sent_v_down")),
            "fc_fwd": r(fc.get("fc_fwd")), "fc_right": r(fc.get("fc_right")),
            "fc_down": r(fc.get("fc_down")), "fc_h": r(fc.get("fc_h")), "flow_q": fc.get("flow_q", ""),
            "roll": r(pt.get("roll_deg"), 1), "pitch": r(pt.get("pitch_deg"), 1),
            "stage": ls.get("stage", "") or "", "reason": ls.get("sent_reason", "") or "",
            "note": ls.get("note", "") or "",
        }
        key = tuple(v for k, v in row.items() if k not in self._VOLATILE)
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
    def _tracking(rows: List[dict]) -> List[str]:
        """How the Pixhawk followed the sent horizontal velocity: delay that best matches
        fc to sent, and the gain at that delay (fc ~ gain * sent(t - delay))."""
        out = []
        for ax in ("fwd", "right"):
            pts = [(r["t"], r["sent_" + ax], r["fc_" + ax]) for r in rows
                   if r["sent_" + ax] != "" and r["fc_" + ax] != ""]
            if len(pts) < 40:
                continue
            best = None
            for lag10 in range(0, 21):                      # 0 .. 2.0 s
                lag, j, num, den, ff, n = lag10 / 10.0, 0, 0.0, 0.0, 0.0, 0
                for t, _, fcv in pts:
                    while j + 1 < len(pts) and pts[j + 1][0] <= t - lag + 0.02:
                        j += 1
                    if pts[j][0] > t - lag + 0.02:
                        continue
                    s = pts[j][1]
                    num, den, ff, n = num + s * fcv, den + s * s, ff + fcv * fcv, n + 1
                if n > 20 and den > 1e-6 and ff > 1e-9:
                    corr = num / math.sqrt(den * ff)
                    if best is None or corr > best[1]:
                        best = (lag, corr, num / den)
            if best:
                over = max(abs(p[2]) for p in pts)
                out.append(f"fc follows sent_{ax}: delay ~{best[0]:.1f} s, gain {best[2]:.2f}, "
                           f"corr {best[1]:.2f}, max fc speed {100 * over:.1f} cm/s")
        return out

    @classmethod
    def summarize(cls, rows: List[dict]) -> str:
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
        # time per height band (where it spends the flight)
        bands = [(0.0, 0.2), (0.2, 0.3), (0.3, 0.4), (0.4, 0.7), (0.7, 9.0)]
        bt = []
        for lo, hi in bands:
            dur = sum(b["t"] - a["t"] for a, b in zip(rows, rows[1:])
                      if a["h"] != "" and lo <= a["h"] < hi)
            errs = [r["err"] for r in rows if r["h"] != "" and r["err"] != "" and lo <= r["h"] < hi]
            if dur > 0:
                bt.append(f"{lo:.1f}-{hi if hi < 9 else 'up'}: {dur:.1f} s"
                          + (f" (err mean {100 * sum(errs) / len(errs):.1f} cm)" if errs else ""))
        out.append("time by height: " + ", ".join(bt))
        out += cls._tracking(rows)
        sink = [(a, b) for a, b in zip(rows, rows[1:])
                if a["sent_down"] != "" and a["sent_down"] < -0.02 and a["h"] != "" and b["h"] != ""]
        if sink:
            dh = sum(b["h"] - a["h"] for a, b in sink)
            dt = sum(b["t"] - a["t"] for a, b in sink)
            out.append(f"asked to climb for {dt:.1f} s: lidar changed {100 * dh:+.1f} cm")
        commit = next((r for r in rows if r["committed"] in (1, "1")), None)
        if commit:
            out.append(f"committed at h {commit['h']} m, err {commit['err']} m, t {commit['t']} s")
        notes: Dict[str, int] = {}
        for r in rows:
            n = r["note"].split(":")[0] if r["note"] else ""
            if n:
                notes[n] = notes.get(n, 0) + 1
        out.append("notes: " + ", ".join(f"{k} x{v}" for k, v in sorted(notes.items(), key=lambda x: -x[1])))
        last = next((r for r in reversed(rows) if r["det"]), None)
        if last:
            out.append(f"last seen: h {last['h']} m, err {last['err']} m "
                       f"(fwd {last['e_fwd']}, right {last['e_right']}), phase {last['phase']}")
        out.append(f"end phase {rows[-1]['phase']}, note '{rows[-1]['note']}'")
        return "\n".join(out) + "\n"

    def close(self):
        with self._lock:
            self._close_locked()
