"""Landing control law: target position -> velocity setpoint (drone frame), with phases.

Pure logic, no I/O: the camera node feeds it every frame and publishes the result. In
SHADOW mode (the only mode for now) nothing is sent to the Pixhawk; the pilot flies Loiter
and the panel shows what the law WOULD command, to check directions and gains first.

Input  (each frame): target (forward_m, right_m) of the touchdown point relative to the
                     drone centre, level frame (tilt-compensated), or None if not valid;
                     h_m = lidar height (vertical), or None.
Output: v_fwd, v_right (m/s, + = forward/right), v_down (m/s, + = DOWN), phase, err_m.

Phases:
    SEARCH    no valid target: hold (v = 0); climb slowly if it was lost low during descent
    ALIGN     move over the target; descend only after |err| < align_tol for align_hold_s
    DESCEND   keep aligning, descend at descend_speed; pause (v_down = 0) if |err| > pause_err
    FINAL     below final_h: slower descent, same pause rule
    TOUCHDOWN below touchdown_h: v = 0 (touchdown method still to be decided by the pilot)
    HOLD      ALIGN took longer than align_timeout_s: v = 0 until reset / lifted
TOUCHDOWN and HOLD re-arm when the drone is lifted rearm_dh above where it stopped (hand tests).
Target lost < lost_hold_s: keep the phase, v = 0 (brake). Lost >= lost_search_s: SEARCH.
"""

import math
from dataclasses import dataclass, fields


@dataclass
class LandingParams:
    kp: float = 0.5                 # m/s per m of error (0.5 -> 10 cm error = 5 cm/s)
    v_max: float = 0.30             # horizontal speed limit, m/s
    deadband: float = 0.02          # error below this -> no horizontal command, m
    align_tol: float = 0.05         # "aligned" when |err| below this, m
    align_hold_s: float = 2.0       # ...for this long before descending
    align_timeout_s: float = 30.0   # ALIGN longer than this -> HOLD
    descend_speed: float = 0.15     # m/s
    pause_err: float = 0.10         # stop descending while |err| above this, m
    final_h: float = 0.30           # FINAL below this height, m
    final_speed: float = 0.10       # m/s
    touchdown_h: float = 0.08       # TOUCHDOWN below this height, m
    lost_hold_s: float = 0.5        # brake (v = 0) while lost less than this
    lost_search_s: float = 3.0      # back to SEARCH after this
    climb_speed: float = 0.15       # SEARCH after a loss low down: climb at this, m/s
    search_h: float = 0.8           # ...up to this height, m
    rearm_dh: float = 0.30          # TOUCHDOWN/HOLD re-arm (-> SEARCH) when lifted this much, m

    @classmethod
    def from_dict(cls, d):
        names = {f.name for f in fields(cls)}
        return cls(**{k: float(v) for k, v in (d or {}).items() if k in names})


class LandingLaw:
    def __init__(self, params=None):
        self.p = params or LandingParams()
        self.reset()

    def reset(self):
        self.phase = "SEARCH"
        self.phase_t = None
        self.aligned_since = None
        self.last_valid_t = None
        self.climb = False
        self.stop_h = None      # height where TOUCHDOWN/HOLD started

    def _set(self, phase, now, h=None):
        if phase != self.phase:
            self.phase, self.phase_t = phase, now
            self.aligned_since = None
            if phase in ("HOLD", "TOUCHDOWN"):
                self.stop_h = h

    def _lifted(self, h):
        return h is not None and self.stop_h is not None and h > self.stop_h + self.p.rearm_dh

    def _horizontal(self, f, r):
        e = math.hypot(f, r)
        if e < self.p.deadband:
            return 0.0, 0.0
        vf, vr = self.p.kp * f, self.p.kp * r
        n = math.hypot(vf, vr)
        if n > self.p.v_max:
            vf, vr = vf * self.p.v_max / n, vr * self.p.v_max / n
        return vf, vr

    def step(self, target, h_m, now):
        p = self.p
        if self.phase_t is None:
            self.phase_t = now
        valid = target is not None and h_m is not None
        out = {"phase": self.phase, "v_fwd": 0.0, "v_right": 0.0, "v_down": 0.0,
               "err_m": None, "h_m": h_m, "lost_s": None, "note": ""}

        if not valid:
            lost = None if self.last_valid_t is None else now - self.last_valid_t
            out["lost_s"] = None if lost is None else round(lost, 2)
            if self.phase in ("DESCEND", "FINAL") and lost is not None and lost >= p.lost_search_s:
                self.climb = True
            if lost is None or lost >= p.lost_search_s:
                if self.phase not in ("HOLD", "TOUCHDOWN"):
                    self._set("SEARCH", now)
            if self.phase == "SEARCH" and self.climb and h_m is not None and h_m < p.search_h:
                out["v_down"] = -p.climb_speed
                out["note"] = "lost low: climbing"
            elif self.phase == "SEARCH" and self.climb:
                self.climb = False
            if self.phase in ("HOLD", "TOUCHDOWN") and self._lifted(h_m):
                self._set("SEARCH", now)
            out["phase"] = self.phase
            if not out["note"]:
                out["note"] = "no target" if lost is None or lost >= p.lost_hold_s else "lost: brake"
            return self._round(out)

        f, r = target
        e = math.hypot(f, r)
        self.last_valid_t = now
        self.climb = False
        out["err_m"] = e

        if self.phase in ("HOLD", "TOUCHDOWN"):
            if self._lifted(h_m):
                self._set("SEARCH", now)
            else:
                out["phase"] = self.phase
                out["note"] = "align timeout: pilot" if self.phase == "HOLD" else "touchdown: pilot LAND/disarm"
                return self._round(out)
        if self.phase == "SEARCH":
            self._set("ALIGN", now)

        out["v_fwd"], out["v_right"] = self._horizontal(f, r)

        if self.phase == "ALIGN":
            if e < p.align_tol:
                self.aligned_since = self.aligned_since or now
                if now - self.aligned_since >= p.align_hold_s:
                    self._set("DESCEND", now)
            else:
                self.aligned_since = None
                if now - self.phase_t > p.align_timeout_s:
                    self._set("HOLD", now, h_m)
                    out["v_fwd"] = out["v_right"] = 0.0
        if self.phase == "DESCEND":
            out["v_down"] = p.descend_speed if e <= p.pause_err else 0.0
            if h_m < p.final_h:
                self._set("FINAL", now)
        if self.phase == "FINAL":
            out["v_down"] = p.final_speed if e <= p.pause_err else 0.0
            if h_m < p.touchdown_h:
                self._set("TOUCHDOWN", now, h_m)
                out["v_fwd"] = out["v_right"] = out["v_down"] = 0.0
        if self.phase in ("DESCEND", "FINAL") and e > p.pause_err:
            out["note"] = "error > pause: holding height"
        out["phase"] = self.phase
        return self._round(out)

    @staticmethod
    def _round(out):
        for k in ("v_fwd", "v_right", "v_down", "err_m", "h_m"):
            if out.get(k) is not None:
                out[k] = round(float(out[k]), 4)
        return out
