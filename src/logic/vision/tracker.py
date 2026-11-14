"""Per-frame vision pipeline: detect markers -> pose -> select and smooth the target.

Pure processing (no camera, threads or network), so it can be driven by the
onboard VisionNode, by recorded clips, or by tests.

    tracker = TargetTracker(detector, camera_model, sizes_m={10: 0.1, 25: 0.1, 32: 0.1})
    measurements = tracker.update(gray, stamp)   # every reliable marker in this frame
    target = tracker.target(now)                 # dict for the ground station / controller
"""

import time

import cv2
import numpy as np

from .pose import estimate_all


class _Track:
    """Smoothed state of one marker ID."""

    def __init__(self, m):
        self.last = m              # latest raw measurement
        self.tvec = np.array(m.tvec, float)
        self.seen_at = m.stamp
        self.frames = 1

    def update(self, m, alpha, max_jump_m):
        jump = np.linalg.norm(np.asarray(m.tvec) - self.tvec)
        if jump > max_jump_m:
            self.tvec = np.array(m.tvec, float)       # real move or a new pass: restart
        else:
            self.tvec = alpha * np.asarray(m.tvec) + (1 - alpha) * self.tvec
        self.last, self.seen_at = m, m.stamp
        self.frames += 1


class TargetTracker:
    """Pure Python vision target processing (OpenCV ArUco)."""

    def __init__(self, detector=None, camera_model=None, sizes_m=None,
                 max_reproj_err_px=2.0, primary_ids=None, smoothing=0.6,
                 max_jump_m=0.30, hold_s=0.5, camera_matrix=None):
        # `camera_matrix` is accepted for compatibility with the old stub's signature.
        self.detector = detector
        self.camera = camera_model
        self.sizes_m = sizes_m or {}
        self.max_reproj_err_px = max_reproj_err_px
        self.primary_ids = list(primary_ids or [])
        self.alpha = float(min(max(smoothing, 0.01), 1.0))
        self.max_jump_m = max_jump_m
        self.hold_s = hold_s
        self.tracks = {}            # marker_id -> _Track
        self.last_frame_stamp = None

    # ------------------------------------------------------------------ core
    def update(self, gray, stamp):
        """Process one grey frame. Returns this frame's reliable measurements."""
        found = self.detector.detect(gray)
        ms = estimate_all(found, self.sizes_m, self.camera, stamp, self.max_reproj_err_px)
        good = [m for m in ms if m.reliable]
        for m in good:
            if m.marker_id in self.tracks:
                self.tracks[m.marker_id].update(m, self.alpha, self.max_jump_m)
            else:
                self.tracks[m.marker_id] = _Track(m)
        self.last_frame_stamp = stamp
        # Forget markers not seen for longer than hold_s.
        for marker_id in [i for i, t in self.tracks.items() if stamp - t.seen_at > self.hold_s]:
            del self.tracks[marker_id]
        return good

    def _primary(self):
        if not self.tracks:
            return None
        for marker_id in self.primary_ids:
            if marker_id in self.tracks:
                return self.tracks[marker_id]
        # No priority list match: the most recently seen, then the closest.
        return max(self.tracks.values(), key=lambda t: (t.seen_at, -np.linalg.norm(t.tvec)))

    def target(self, now=None):
        """The selected target as a dict (Bridge_WIFI aruco_pose format + extras).

        `x, y, z, distance` are smoothed; `distance_raw` is the latest raw value.
        `age_ms` = how old the measurement is (capture time -> now); a held
        (lost) marker keeps detected=True until hold_s, with a growing age_ms.
        """
        now = time.monotonic() if now is None else now
        track = self._primary()
        if track is None:
            return {"id": None, "detected": False, "x": 0.0, "y": 0.0, "z": 0.0,
                    "distance": None, "visible_ids": []}
        m = track.last
        d = m.to_dict()
        x, y, z = (float(v) for v in track.tvec)
        d.update({
            "x": round(x, 4), "y": round(y, 4), "z": round(z, 4),
            "distance": round(float(np.linalg.norm(track.tvec)), 4),
            "distance_raw": round(float(m.distance), 4),
            "age_ms": round((now - m.stamp) * 1000.0, 1),
            "in_view": bool(self.last_frame_stamp is not None
                            and m.stamp == self.last_frame_stamp),
            "visible_ids": sorted(i for i, t in self.tracks.items()
                                  if t.seen_at == self.last_frame_stamp),
        })
        return d

    # ------------------------------------------------------- old stub API
    def process_frame(self, raw_data, width=None, height=None):
        """Compatibility with the original stub: image in, target dict out."""
        if raw_data is None or getattr(raw_data, "size", 0) == 0:
            return {"x": 0.0, "y": 0.0, "z": 0.0}
        gray = raw_data if raw_data.ndim == 2 else cv2.cvtColor(raw_data, cv2.COLOR_BGR2GRAY)
        self.update(gray, time.monotonic())
        return self.target()
