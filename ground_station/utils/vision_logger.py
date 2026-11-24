# file: ground_station/utils/vision_logger.py
import csv
import os
import threading
import time
from datetime import datetime
from typing import Any, Callable, Dict, Optional, TextIO, Tuple


class VisionExperimentLogger:
    """Records the onboard vision packets to CSV on the ground station.

    Recording is automatic: a file is open while the Pixhawk is ARMED and in LOITER or GUIDED
    (read from the ground station's MAVLink receiver). Any other mode, disarm or a lost MAVLink
    link closes it after CLOSE_GRACE_S. The Pi's `visionlog` command only gives the file a name;
    it never records outside LOITER/GUIDED. Nothing is written on the Pi. One row per received
    vision packet (~30 Hz) -> experiments/vision/<YYYY-MM-DD>/<name>.csv."""

    RECORD_MODES = {4: "GUIDED", 5: "LOITER"}   # ArduCopter custom_mode
    CLOSE_GRACE_S = 1.5                          # so a brief mode flicker does not split files
    HB_MAX_AGE_S = 2.5

    FIELDS = ("source", "approval", "du_px", "dv_px", "nx", "ny", "h_m", "x_m", "y_m",
              "fwd_raw_m", "right_raw_m", "fwd_m", "right_m", "h_vert_m", "roll_deg", "pitch_deg",
              "att_age_s", "age_s", "pad_fwd_m", "pad_right_m", "h_cam_m")
    SHADOW = ("phase", "v_fwd", "v_right", "v_down", "err_m", "target", "note",
              "mode", "sent", "sent_v_fwd", "sent_v_right", "sent_v_down", "sent_reason")
    HEADERS = (("gs_time_iso", "elapsed_s", "detected", "tracking_state") + FIELDS
               + tuple("law_" + k for k in SHADOW) + ("fc_mode",))

    def __init__(self, base_dir: Optional[str] = None,
                 mode_source: Optional[Callable[[], Optional[Tuple[bool, int]]]] = None):
        if base_dir is None:
            repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
            base_dir = os.path.join(repo_root, "experiments", "vision")
        self.base_dir = base_dir
        self.mode_source = mode_source or self._mavlink_mode
        self._lock = threading.Lock()
        self._file: Optional[TextIO] = None
        self._writer = None
        self._name = ""
        self._path = ""
        self._t0 = 0.0
        self._rows = 0
        self._left_since: Optional[float] = None

    @classmethod
    def _mavlink_mode(cls) -> Optional[Tuple[bool, int]]:
        """(armed, custom_mode) from the ground station MAVLink receiver; None if missing/stale."""
        from ground_station.receivers.nodes.mavlink_rx import MavlinkReceiver
        rx = getattr(MavlinkReceiver, "_active_instance", None)
        if rx is None or time.time() - getattr(rx, "last_seen", 0.0) > cls.HB_MAX_AGE_S:
            return None
        hb = rx.get_packet("heartbeat") or {}
        if "custom_mode" not in hb:
            return None
        return bool(hb.get("armed")), int(hb.get("custom_mode", -1))

    def process_packets(self, packets: Dict[str, Any], fresh=None, now: Optional[float] = None):
        """packets = latest value of every packet; fresh = names that came in THIS datagram.
        A row is written only when pixel_target is fresh, so a cached (stale) value is never
        repeated in the CSV."""
        now = time.time() if now is None else now
        flag = packets.get("vision_log") or {}
        rec, name = bool(flag.get("rec")), str(flag.get("name") or "")
        try:
            state = self.mode_source()
        except Exception:
            state = None
        allowed = bool(state) and state[0] and state[1] in self.RECORD_MODES
        mode_name = self.RECORD_MODES.get(state[1], f"MODE_{state[1]}") if state else ""
        with self._lock:
            if allowed:
                self._left_since = None
                if self._file is None:
                    self._open_locked(name if rec and name else datetime.now().strftime("auto_%H-%M-%S"))
            elif self._file is not None:
                self._left_since = self._left_since or now
                if now - self._left_since >= self.CLOSE_GRACE_S:
                    self._close_locked()
            if allowed and self._file is not None and (fresh is None or "pixel_target" in fresh):
                self._write_locked(packets, now, mode_name)

    def _open_locked(self, name: str):
        day_dir = os.path.join(self.base_dir, datetime.now().strftime("%Y-%m-%d"))
        os.makedirs(day_dir, exist_ok=True)
        self._path, n = os.path.join(day_dir, f"{name}.csv"), 2
        while os.path.exists(self._path):          # never overwrite an earlier segment
            self._path, n = os.path.join(day_dir, f"{name}_{n}.csv"), n + 1
        self._file = open(self._path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADERS)
        self._name, self._t0, self._rows = name, time.time(), 0
        print(f"[VISION_LOGGER] Recording -> {self._path}")

    def _write_locked(self, packets: Dict[str, Any], now: float, mode_name: str):
        pt = packets.get("pixel_target") or {}
        status = packets.get("status") or {}
        row = [datetime.now().isoformat(timespec="milliseconds"), f"{now - self._t0:.3f}",
               int(bool(pt.get("detected"))), status.get("tracking_state", "")]
        row += ["" if pt.get(k) is None else pt.get(k) for k in self.FIELDS]
        ls = packets.get("landing_shadow") or {}
        row += ["" if ls.get(k) is None else ls.get(k) for k in self.SHADOW]
        row.append(mode_name)
        self._writer.writerow(row)
        self._rows += 1

    def _close_locked(self):
        if self._file is None:
            return
        self._file.close()
        print(f"[VISION_LOGGER] Saved {self._rows} rows -> {self._path}")
        self._file, self._writer = None, None

    def close(self):
        with self._lock:
            self._close_locked()
