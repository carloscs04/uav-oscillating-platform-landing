# file: ground_station/utils/plnd_logger.py
import csv
import math
import os
import threading
import time
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional, TextIO, Tuple


class PrecLandLogger:
    """Precision-landing log (ArduPilot LAND mode + the Pi's LANDING_TARGET), written on the
    ground station. Replaces asking for the Pixhawk .bin: what the camera saw of the pad and
    how the drone moved, only while ARMED in LAND or GUIDED, one row per NEW vision frame.
    -> the flight's folder: experiments/flights/<date>/<HH-MM-SS>_<codename>/
       flight_<HH-MM-SS>_<codename>_plnd.csv + _plnd_summary.txt
    (separate from the big flight csv on purpose; no flight being recorded -> <date>/plnd_<HH-MM-SS>.csv).

    Columns: id = chosen marker, pad_ids = pad markers averaged for the touchdown point,
    h = camera/lidar height (m), e_fwd/e_right = touchdown point seen by the camera
    (m, + = ahead/right of the drone), err = its distance, lt_* = LANDING_TARGET sent (body FRD),
    age_ms = camera frame read -> measurement published (Pi processing latency),
    fc_* = Pixhawk velocity in the body frame (m/s), fc_h = EKF height, roll/pitch (deg)."""

    LAND = 9
    GUIDED = 4      # also logged: POSITION HOLD (poshold) is judged with the same columns
    CLOSE_GRACE_S = 1.5
    HEADERS = ("t", "det", "id", "pad_ids", "h", "e_fwd", "e_right", "err", "lt_x", "lt_y", "lt_z", "age_ms",
               "fc_fwd", "fc_right", "fc_down", "fc_h", "flow_q", "roll", "pitch")
    _VOLATILE = {"t", "fc_fwd", "fc_right", "fc_down", "fc_h", "flow_q"}

    def __init__(self, base_dir: Optional[str] = None,
                 mode_source: Optional[Callable[[], Optional[Tuple[bool, int]]]] = None,
                 fc_source: Optional[Callable[[], Dict[str, Any]]] = None,
                 session_source: Optional[Callable[[], Optional[Tuple[str, str]]]] = None):
        if base_dir is None:
            repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
            base_dir = os.path.join(repo_root, "experiments", "flights")
        self.base_dir = base_dir
        if mode_source is None:
            from ground_station.utils.vision_logger import VisionExperimentLogger
            mode_source = VisionExperimentLogger._mavlink_mode
        if fc_source is None:
            from ground_station.utils.pid_logger import PidTuningLogger
            fc_source = PidTuningLogger._mavlink_fc
        if session_source is None:
            from ground_station.utils.flight_logger import FlightExperimentLogger
            session_source = lambda: FlightExperimentLogger.active_session
        self.mode_source = mode_source
        self.fc_source = fc_source
        self.session_source = session_source
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
        allowed = bool(state) and state[0] and state[1] in (self.LAND, self.GUIDED)
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
        try:
            session = self.session_source()
        except Exception:
            session = None
        if session:
            day_dir, name = session[0], f"{session[1]}_plnd"
        else:
            day_dir = os.path.join(self.base_dir, datetime.now().strftime("%Y-%m-%d"))
            name = datetime.now().strftime("plnd_%H-%M-%S")
        os.makedirs(day_dir, exist_ok=True)
        n = 2
        self._path = os.path.join(day_dir, f"{name}.csv")
        while os.path.exists(self._path):
            self._path, n = os.path.join(day_dir, f"{name}_{n}.csv"), n + 1
        self._file = open(self._path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADERS)
        self._t0, self._last_key, self._rows = now, None, []
        print(f"[PLND_LOGGER] Recording -> {self._path}")

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
        try:
            fc = self.fc_source() or {}
        except Exception:
            fc = {}
        r = self._r
        src = str(pt.get("source") or "")
        det = bool(pt.get("detected"))
        ef, er = (r(pt.get("pad_fwd_m")), r(pt.get("pad_right_m"))) if det else ("", "")
        row = {
            "t": round(now - self._t0, 2), "det": int(det),
            "id": src.split("#")[-1] if "#" in src else "", "pad_ids": pt.get("pad_ids", "") if det else "",
            "h": r(pt.get("h_m")),
            "e_fwd": ef, "e_right": er,
            "err": round(math.hypot(ef, er), 4) if ef != "" and er != "" else "",
            "lt_x": r(pt.get("lt_x")) if det else "", "lt_y": r(pt.get("lt_y")) if det else "",
            "lt_z": r(pt.get("lt_z")) if det else "",
            "age_ms": round(1000 * float(pt["frame_age_s"])) if det and pt.get("frame_age_s") is not None else "",
            "fc_fwd": r(fc.get("fc_fwd")), "fc_right": r(fc.get("fc_right")),
            "fc_down": r(fc.get("fc_down")), "fc_h": r(fc.get("fc_h")), "flow_q": fc.get("flow_q", ""),
            "roll": r(pt.get("roll_deg"), 1), "pitch": r(pt.get("pitch_deg"), 1),
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
            print(f"[PLND_LOGGER] summary failed: {e}")
        print(f"[PLND_LOGGER] Saved {len(self._rows)} rows -> {self._path}")

    @staticmethod
    def summarize(rows: List[dict]) -> str:
        if not rows:
            return "no rows\n"
        out = [f"LAND duration {rows[-1]['t']:.1f} s, {len(rows)} frames"]
        seen = [r for r in rows if r["det"]]
        lost = sum(1 for a, b in zip(rows, rows[1:]) if a["det"] and not b["det"])
        out.append(f"marker seen in {100 * len(seen) / len(rows):.0f}% of frames, lost {lost} times")
        hs = [r for r in rows if r["fc_h"] != ""]
        if hs:
            h0 = hs[0]["fc_h"]
            start = next((r for r in hs if r["fc_h"] < h0 - 0.05), None)
            if start:
                out.append(f"held {start['t']:.1f} s at {h0:.2f} m before descending (centring)")
                end = next((r for r in hs if r["fc_h"] < 0.15), hs[-1])
                if end["t"] > start["t"]:
                    out.append(f"descent {start['fc_h'] - end['fc_h']:.2f} m in {end['t'] - start['t']:.1f} s "
                               f"({100 * (start['fc_h'] - end['fc_h']) / (end['t'] - start['t']):.1f} cm/s)")
        out.append("height band   err_mean_cm  err_max_cm  side_speed_mean_cm/s")
        for lo, hi in ((0.7, 9.0), (0.4, 0.7), (0.2, 0.4), (0.0, 0.2)):
            sel = [r for r in rows if r["h"] != "" and lo <= r["h"] < hi]
            errs = [r["err"] for r in sel if r["err"] != ""]
            sp = [math.hypot(r["fc_fwd"], r["fc_right"]) for r in sel
                  if r["fc_fwd"] != "" and r["fc_right"] != ""]
            if not sel:
                continue
            band = f"{lo:.1f}-{hi:.1f} m" if hi < 9 else f">{lo:.1f} m"
            em = f"{100 * sum(errs) / len(errs):.1f}" if errs else "-"
            ex = f"{100 * max(errs):.1f}" if errs else "-"
            ms = f"{100 * sum(sp) / len(sp):.1f}" if sp else "-"
            out.append(f"{band:<12}  {em:>11}  {ex:>10}  {ms:>20}")
        ages = sorted(r["age_ms"] for r in seen if r.get("age_ms", "") != "")
        if ages:
            out.append(f"Pi latency frame->target: median {ages[len(ages) // 2]} ms, "
                       f"p90 {ages[int(0.9 * (len(ages) - 1))]} ms (+ camera exposure/USB, not measured)")
        last = seen[-1] if seen else None
        if last:
            out.append(f"last seen (~touchdown error): h {last['h']} m, err {100 * last['err']:.1f} cm "
                       f"(fwd {100 * last['e_fwd']:+.1f}, right {100 * last['e_right']:+.1f} cm)"
                       if last["err"] != "" else f"last seen: h {last['h']} m (no pad point)")
        for lo, hi in ((0.4, 9.0), (0.2, 0.4), (0.0, 0.2)):
            q = [int(r["flow_q"]) for r in rows if r.get("flow_q", "") not in ("", None)
                 and r["fc_h"] != "" and lo <= (r["h"] if r["h"] != "" else r["fc_h"]) < hi]
            if q:
                out.append(f"flow quality {lo:.1f}-{hi:.1f} m: mean {sum(q) / len(q):.0f}, min {min(q)}")
        low = [r for r in rows if r["h"] != "" and r["h"] < 0.2 and r["fc_fwd"] != ""]
        if low:
            r0 = low[-1]
            out.append(f"side speed at touchdown: {100 * math.hypot(r0['fc_fwd'], r0['fc_right']):.1f} cm/s")
        return "\n".join(out) + "\n"

    def close(self):
        with self._lock:
            self._close_locked()
