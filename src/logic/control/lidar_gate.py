# file: src/logic/control/lidar_gate.py
"""Lidar spike gate: a slot in the platform (or anything that makes the lidar see much farther
or nearer for a moment, e.g. 45 45 95 45 45) must not look like a sudden climb.

Every OK lidar sample is compared with the PREDICTION = last good height + the Pixhawk's
vertical speed x dt. Real relative height changes smoothly (even with the platform moving),
so a step larger than jump_m is suspect:

  * camera in view (ArUco height, corrected to the lidar scale): it is the referee.
    lidar ~ camera  -> the step is real (e.g. flying onto the table), accepted at once.
    lidar != camera -> slot: rejected, the output is the camera height, for as long as needed.
  * no camera: output = prediction (dead reckoning on the Pixhawk speed), at most est_max_s.
  * after that: UNRELIABLE -> no height at all (the Pi stops sending the rangefinder, the law
    gets no height and does not descend). It ends when the lidar comes back near the
    prediction, the camera confirms, or the lidar stays steady for settle_s (a real new level
    with nothing else to check it, e.g. manual flight without markers).

Gaps longer than reset_gap_s (sensor out of range, NO_TARGET) restart the gate: the first
sample after them is accepted as is. Camera vs lidar: the camera gives the vertical height,
the lidar the slant range; under ~15 deg of tilt the difference is inside cam_agree_m."""

from dataclasses import dataclass


@dataclass
class LidarGateParams:
    enabled: bool = True
    jump_m: float = 0.10        # |lidar - prediction| above this = suspect (99.9 % of flight samples < 9 cm)
    cam_agree_m: float = 0.07   # lidar within this of the camera -> the step is real
    est_max_s: float = 2.0      # max time on the prediction alone (no camera), then UNRELIABLE
    settle_s: float = 1.0       # UNRELIABLE: lidar steady this long -> accepted as the new level
    settle_band_m: float = 0.03
    reset_gap_s: float = 0.5    # no OK sample this long -> restart (first sample accepted)
    cam_k: float = 1.077        # camlidar 2026-11-25: cam = 1.077 * lidar - 2.5 cm (both markers)
    cam_c_m: float = -0.025


class LidarGate:
    OK, CAM, EST, UNRELIABLE = "OK", "REJECT_CAM", "REJECT_EST", "UNRELIABLE"

    def __init__(self, params=None):
        self.p = params or LidarGateParams()
        self.reset()
        self.rejected_total = 0      # samples not used since start
        self.events = 0              # number of rejection episodes

    def reset(self):
        self.h = None                # current output estimate (m)
        self.t = None                # time of the last OK sample
        self.status = self.OK
        self.blind_since = None      # start of rejecting WITHOUT camera
        self._settle = None          # (t0, ref) lidar steadiness while UNRELIABLE

    def cam_to_lidar(self, cam_h):
        return None if cam_h is None else (cam_h - self.p.cam_c_m) / self.p.cam_k

    def step(self, raw, now, vz_up=None, cam_h=None):
        """raw lidar m (status OK), now s, vz_up m/s (+ = climbing) or None, cam_h = camera height
        (m, as measured, uncorrected) or None. Returns the height to use (m) or None."""
        p = self.p
        if not p.enabled:
            self.reset()
            self.h, self.t = raw, now
            return raw
        if self.t is None or self.h is None or now - self.t > p.reset_gap_s:
            self.reset()
            self.h, self.t = raw, now
            return raw
        dt = max(0.0, now - self.t)
        self.t = now
        pred = self.h + (vz_up or 0.0) * dt
        cam = self.cam_to_lidar(cam_h)
        if abs(raw - pred) <= p.jump_m or (cam is not None and abs(raw - cam) <= p.cam_agree_m):
            return self._accept(raw)
        # suspect step
        if self.status == self.OK:
            self.events += 1
        self.rejected_total += 1
        if cam is not None:
            self.status, self.h, self.blind_since, self._settle = self.CAM, cam, None, None
            return cam
        self.h = pred
        if self.blind_since is None:
            self.blind_since = now
        if now - self.blind_since <= p.est_max_s:
            self.status = self.EST
            return pred
        self.status = self.UNRELIABLE
        if self._settle is None or abs(raw - self._settle[1]) > p.settle_band_m:
            self._settle = (now, raw)
        elif now - self._settle[0] >= p.settle_s:
            return self._accept(raw)
        return None

    def _accept(self, raw):
        self.h, self.status, self.blind_since, self._settle = raw, self.OK, None, None
        return raw

    def snapshot(self):
        return {"status": self.status, "rejected_total": self.rejected_total, "events": self.events,
                "enabled": self.p.enabled}
