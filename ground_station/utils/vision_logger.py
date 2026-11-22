# file: ground_station/utils/vision_logger.py
import csv
import os
import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional, TextIO


class VisionExperimentLogger:
    """Records the onboard vision packets to CSV on the ground station while the Pi's CLI
    `visionlog` is running (packet "vision_log": {"rec": True, "name": ...}).

    Nothing is written on the Pi: it only raises the flag. One row per received vision
    packet (~30 Hz) -> experiments/vision/<YYYY-MM-DD>/<name>.csv."""

    FIELDS = ("source", "approval", "du_px", "dv_px", "nx", "ny", "h_m", "x_m", "y_m",
              "fwd_raw_m", "right_raw_m", "fwd_m", "right_m", "h_vert_m", "roll_deg", "pitch_deg",
              "att_age_s", "age_s")
    HEADERS = ("gs_time_iso", "elapsed_s", "detected", "tracking_state") + FIELDS

    def __init__(self, base_dir: Optional[str] = None):
        if base_dir is None:
            repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
            base_dir = os.path.join(repo_root, "experiments", "vision")
        self.base_dir = base_dir
        self._lock = threading.Lock()
        self._file: Optional[TextIO] = None
        self._writer = None
        self._name = ""
        self._path = ""
        self._t0 = 0.0
        self._rows = 0

    def process_packets(self, packets: Dict[str, Any]):
        flag = packets.get("vision_log") or {}
        rec, name = bool(flag.get("rec")), str(flag.get("name") or "")
        with self._lock:
            if rec and (self._file is None or name != self._name):
                self._close_locked()
                self._open_locked(name or datetime.now().strftime("visionlog_%H-%M-%S"))
            elif not rec and self._file is not None:
                self._close_locked()
            if self._file is not None:
                self._write_locked(packets)

    def _open_locked(self, name: str):
        day_dir = os.path.join(self.base_dir, datetime.now().strftime("%Y-%m-%d"))
        os.makedirs(day_dir, exist_ok=True)
        self._path = os.path.join(day_dir, f"{name}.csv")
        self._file = open(self._path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADERS)
        self._name, self._t0, self._rows = name, time.time(), 0
        print(f"[VISION_LOGGER] Recording -> {self._path}")

    def _write_locked(self, packets: Dict[str, Any]):
        pt = packets.get("pixel_target") or {}
        status = packets.get("status") or {}
        now = time.time()
        row = [datetime.now().isoformat(timespec="milliseconds"), f"{now - self._t0:.3f}",
               int(bool(pt.get("detected"))), status.get("tracking_state", "")]
        row += ["" if pt.get(k) is None else pt.get(k) for k in self.FIELDS]
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
