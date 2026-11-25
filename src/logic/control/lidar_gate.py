# file: src/logic/control/lidar_gate.py
"""Lidar spike gate: a slot in the platform (or anything that makes the lidar see much farther
or nearer for a moment, e.g. 45 45 95 45 45) must not look like a sudden climb.

Every OK lidar sample is compared with the PREDICTION = last good height + the Pixhawk's
vertical speed x dt. Real relative height changes smoothly (even with the platform moving),
so a step larger than jump_m is suspect:

  * camera in view (ArUco height, corrected to the lidar scale): it is the referee. It also
    catches a gradual drift: lidar > cam_disagree_m from the camera for disagree_n samples.
    lidar ~ camera  -> the step is real (e.g. flying onto the table), accepted at once.
    lidar != camera -> slot: rejected, the output is the camera height, for as long as needed.
  * barometer (independent of the lidar, low-passed): prediction = last good lidar + baro
    change since then, and a WINDOW check: the lidar may not change more than win_jump_m in
    win_s beyond what the baro explains (catches a slot entering the lidar cone gradually).
    Replay of the 2026-11-23 platform flights: max 13 cm, so 15 cm never fires there.
  * no camera: output = prediction (baro, else Pixhawk speed), at most est_max_baro_s / est_max_s.
  * after that: UNRELIABLE -> no height at all (the Pi stops sending the rangefinder, the law
    gets no height and does not descend). It ends when the lidar comes back near the
    prediction, the camera confirms, or the lidar stays steady for settle_s (a real new level
    with nothing else to check it, e.g. manual flight without markers).

Active only when armed in LOITER or GUIDED (DistanceState active_fn); otherwise IDLE: the raw
lidar passes (baro climb, ground, ALT_HOLD). `source` says who decided each height.

Gaps longer than reset_gap_s (sensor out of range, NO_TARGET) restart the gate: the first
sample after them is accepted as is. Camera vs lidar: the camera gives the vertical height,
the lidar the slant range; under ~15 deg of tilt the difference is inside cam_agree_m."""

from collections import deque
from dataclasses import dataclass


@dataclass
class LidarGateParams:
    enabled: bool = True
    jump_m: float = 0.10        # |lidar - prediction| above this = suspect (99.9 % of flight samples < 9 cm)
    cam_agree_m: float = 0.07   # lidar within this of the camera -> the step is real
    cam_disagree_m: float = 0.10  # no step, but lidar this far from the camera ...
    disagree_n: int = 3           # ... for this many samples in a row -> camera corrects
                                  # (a slot entering the lidar cone gradually, 2026-11-24 bench)
    est_max_s: float = 2.0      # max time on the prediction alone (no camera), then UNRELIABLE
    est_max_baro_s: float = 4.0 # ... with the barometer as the prediction
    baro_tau_s: float = 0.5     # barometer low-pass
    win_s: float = 0.3          # window check: lidar change over win_s ...
    win_jump_m: float = 0.15    # ... minus baro change over win_s above this = suspect
    settle_s: float = 3.0       # UNRELIABLE: lidar steady this long -> accepted as the new level
                                # (long: the camera should be the one that corrects)
    settle_band_m: float = 0.03
    reset_gap_s: float = 0.5    # no OK sample this long -> restart (first sample accepted)
    cam_k: float = 1.077        # camlidar 2026-11-25: cam = 1.077 * lidar - 2.5 cm (both markers)
    cam_c_m: float = -0.025


class LidarGate:
    OK, CAM, EST, UNRELIABLE, IDLE = "OK", "REJECT_CAM", "REJECT_EST", "UNRELIABLE", "IDLE"
    # who decided the height that is used: LIDAR | CAMERA | ESTIMATE | NONE | LIDAR_RAW (gate idle)

    def __init__(self, params=None):
        self.p = params or LidarGateParams()
        self._bf = self._bt = None   # low-passed baro altitude (m) and its time: kept across resets
        self.reset()
        self.rejected_total = 0      # samples not used since start
        self.events = 0              # number of rejection episodes
        self.on_event = None         # fn(text): one line per decision change (CLI log)
        self.event_seq = 0
        self.event_log = deque(maxlen=20)   # (seq, text): the flight events log picks them up

    def reset(self):
        self.h = None                # current output estimate (m)
        self.t = None                # time of the last OK sample
        self.status = self.OK
        self.blind_since = None      # start of rejecting WITHOUT camera
        self._settle = None          # (t0, ref) lidar steadiness while UNRELIABLE
        self._disagree = 0           # consecutive samples with lidar far from the camera
        self.source = "LIDAR"        # who decided the last output
        self.raw = None              # last raw lidar, m
        self.cam = None              # last camera height converted to the lidar scale, m
        self._anchor = None          # (t, h, baro) of the last accepted lidar sample
        self._hist = deque()         # accepted (t, h, baro), for the window check
        self.pred_src = ""           # "baro" | "speed": what the prediction used

    def _emit(self, text):
        self.event_seq += 1
        self.event_log.append((self.event_seq, text))
        if self.on_event is not None:
            try:
                self.on_event(text)
            except Exception:
                pass

    def idle(self, raw, now):
        """Gate not active (not armed in LOITER/GUIDED): raw lidar passes, state restarts."""
        if self.status != self.IDLE:
            self.reset()
            self.status = self.IDLE
        self.h, self.t, self.raw, self.source = raw, now, raw, "LIDAR_RAW"
        return raw

    def cam_to_lidar(self, cam_h):
        return None if cam_h is None else (cam_h - self.p.cam_c_m) / self.p.cam_k

    def _baro(self, baro_alt, now):
        if baro_alt is None:
            return None
        if self._bf is None or self._bt is None or now - self._bt > 1.0:
            self._bf = float(baro_alt)
        else:
            dt = max(0.0, now - self._bt)
            self._bf += dt / (self.p.baro_tau_s + dt) * (float(baro_alt) - self._bf)
        self._bt = now
        return self._bf

    def step(self, raw, now, vz_up=None, cam_h=None, baro_alt=None):
        """raw lidar m (status OK), now s, vz_up m/s (+ = climbing) or None, cam_h = camera height
        (m, as measured, uncorrected) or None, baro_alt = barometric altitude m (any zero) or None.
        Returns the height to use (m) or None."""
        p = self.p
        cam = self.cam_to_lidar(cam_h)
        bf = self._baro(baro_alt, now)
        self.raw, self.cam = raw, cam
        if not p.enabled:
            return self.idle(raw, now)
        if self.t is None or self.h is None or now - self.t > p.reset_gap_s:
            was = self.status
            self.reset()
            self.h, self.t, self.raw, self.cam = raw, now, raw, cam
            self._remember(raw, now, bf, clear=True)
            if was not in (self.OK, self.IDLE):
                self._emit(f"lidar gap: restart at {raw * 100:.1f} cm")
            return raw
        dt = max(0.0, now - self.t)
        self.t = now
        a = self._anchor
        if bf is not None and a is not None and a[2] is not None:
            pred, self.pred_src = a[1] + (bf - a[2]), "baro"
        else:
            pred, self.pred_src = self.h + (vz_up or 0.0) * dt, "speed"
        far = cam is not None and abs(raw - cam) > p.cam_disagree_m
        self._disagree = self._disagree + 1 if far else 0
        win_bad = False
        if bf is not None:
            old = None
            for e in self._hist:
                if now - e[0] >= p.win_s:
                    old = e
            if old is not None and old[2] is not None and abs(raw - (old[1] + bf - old[2])) > p.win_jump_m:
                win_bad = True
                self._anchor = old           # the samples after it were already drifting:
                while self._hist and self._hist[-1] is not old:
                    self._hist.pop()         # forget them, never anchor on them again
                pred = old[1] + (bf - old[2])
        if abs(raw - pred) <= p.jump_m and self._disagree < p.disagree_n and not win_bad:
            if self.status in (self.CAM, self.EST, self.UNRELIABLE):
                self._emit(f"lidar back to normal: {raw * 100:.1f} cm (LIDAR)")
            return self._accept(raw, now, bf)
        if cam is not None and abs(raw - cam) <= p.cam_agree_m:
            self._emit(f"REAL step {pred * 100:.1f} -> {raw * 100:.1f} cm, confirmed by the CAMERA "
                       f"({cam * 100:.1f} cm): lidar accepted")
            return self._accept(raw, now, bf, clear=True)
        # suspect step
        if self.status in (self.OK, self.IDLE):
            self.events += 1
        self.rejected_total += 1
        if cam is not None:
            if self.status != self.CAM:
                self._emit(f"lidar {raw * 100:.1f} cm REJECTED, CAMERA corrects: {cam * 100:.1f} cm")
            self.status, self.h, self.blind_since, self._settle = self.CAM, cam, None, None
            self.source = "CAMERA"
            return cam
        self.h = pred
        if self.blind_since is None:
            self.blind_since = now
        est_max = p.est_max_baro_s if self.pred_src == "baro" else p.est_max_s
        if now - self.blind_since <= est_max:
            if self.status != self.EST:
                by = "BAROMETER" if self.pred_src == "baro" else "Pixhawk speed"
                self._emit(f"lidar {raw * 100:.1f} cm REJECTED, no ArUco: ESTIMATE {pred * 100:.1f} cm "
                           f"from the {by} (max {est_max:.0f} s, waiting for the camera)")
            self.status, self.source = self.EST, "ESTIMATE"
            return pred
        if self.status != self.UNRELIABLE:
            self._emit(f"UNRELIABLE height: no ArUco for {est_max:.0f} s and lidar {raw * 100:.1f} cm "
                       "does not fit -> no height sent (no descent) until the camera sees a marker")
        self.status, self.source = self.UNRELIABLE, "NONE"
        if self._settle is None or abs(raw - self._settle[1]) > p.settle_band_m:
            self._settle = (now, raw)
        elif now - self._settle[0] >= p.settle_s:
            self._emit(f"lidar steady {p.settle_s:.0f} s at {raw * 100:.1f} cm without camera: accepted as new level")
            return self._accept(raw, now, bf, clear=True)
        return None

    def _accept(self, raw, now=None, bf=None, clear=False):
        self.h, self.status, self.blind_since, self._settle = raw, self.OK, None, None
        self.source = "LIDAR"
        if now is not None:
            self._remember(raw, now, bf, clear)
        return raw

    def _remember(self, h, now, bf, clear=False):
        """Accepted sample -> anchor and window history (cleared after a confirmed new level)."""
        if clear:
            self._hist.clear()
        self._anchor = (now, h, bf)
        self._hist.append(self._anchor)
        while self._hist and now - self._hist[0][0] > self.p.win_s + 0.5:
            self._hist.popleft()

    def snapshot(self):
        return {"status": self.status, "source": self.source, "rejected_total": self.rejected_total,
                "events": self.events, "enabled": self.p.enabled, "baro": self._bf is not None,
                "raw_m": self.raw, "cam_m": None if self.cam is None else round(self.cam, 4),
                "used_m": None if self.source == "NONE" or self.h is None else round(self.h, 4)}
