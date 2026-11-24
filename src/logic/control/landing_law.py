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
    FINAL     below final_h: slower descent, same pause rule; below commit_h no more pauses
              and a lost target no longer stops it (the pad fills the view): blind, straight down
    TOUCHDOWN below touchdown_h: no horizontal command, keep pushing down at final_speed so the
              Pixhawk's land detector spools the motors down; the pilot disarms (ch6)
    HOLD      ALIGN took longer than align_timeout_s: v = 0 until reset / lifted
TOUCHDOWN and HOLD re-arm (-> SEARCH) when the drone is lifted rearm_dh above where it stopped,
or when the target has been out of view for lost_search_s (hand tests: take the drone away).
Target lost < lost_hold_s: keep the phase, v = 0 (brake). Lost >= lost_search_s: SEARCH.

Horizontal: PD. v = kp * err + kd * d(err)/dt (derivative low-pass filtered, d_tau). The D
term opposes the drone's own drift over the pad (flight 2026-11-22: the drone swung +-15 cm
at up to 0.2 m/s while the P command was <= 0.09 m/s).
"""

import math
from dataclasses import dataclass, fields


@dataclass
class LandingParams:
    kp: float = 0.25                # m/s per m of error (0.25 -> 10 cm error = 2.5 cm/s)
                                    # (flight 2026-11-22: Pixhawk velocity lags 2-3 s, 0.5 swung)
    kd: float = 0.4                 # s: m/s per m/s of error rate (damps the drift)
    d_tau: float = 0.25             # low-pass time constant of the error rate, s
    v_max: float = 0.30             # horizontal speed limit, m/s
    deadband: float = 0.02          # error below this -> no horizontal command, m
    align_tol: float = 0.05         # "aligned" when |err| below this, m
    align_hold_s: float = 2.0       # ...for this long before descending
    align_timeout_s: float = 30.0   # ALIGN longer than this -> HOLD
    descend_speed: float = 0.15     # m/s
    pause_err: float = 0.10         # stop descending while |err| above this, m
    final_h: float = 0.30           # FINAL below this height, m
    final_speed: float = 0.10       # m/s
    commit_h: float = 0.30          # below this: no pauses, lost target -> keep descending, m
                                    # (flight 2026-11-22: the marker left the frame at 0.22 m)
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
        self.align_timeout_enabled = True
        self.strict_descent = False     # set by camera_node while the land stage is selected   # off while a GUIDED stage is selected: keep aligning
        self.reset()

    def reset(self):
        self.phase = "SEARCH"
        self.phase_t = None
        self.aligned_since = None
        self.last_valid_t = None
        self.climb = False
        self.stop_h = None      # height where TOUCHDOWN/HOLD started
        self.committed = False  # FINAL below commit_h: descend even without target
        self._prev = None       # (t, f, r) for the error rate
        self.rate = (0.0, 0.0)  # filtered d(err)/dt, m/s

    def _set(self, phase, now, h=None):
        if phase != self.phase:
            self.phase, self.phase_t = phase, now
            self.aligned_since = None
            if phase in ("HOLD", "TOUCHDOWN"):
                self.stop_h = h
            if phase in ("SEARCH", "ALIGN", "HOLD"):
                self.committed = False

    def _lifted(self, h):
        return h is not None and self.stop_h is not None and h > self.stop_h + self.p.rearm_dh

    def _update_rate(self, f, r, now):
        prev, self._prev = self._prev, (now, f, r)
        if prev is None:
            self.rate = (0.0, 0.0)
            return
        dt = now - prev[0]
        if dt <= 1e-3:
            return                          # same frame twice: keep the rate
        if dt > 0.3:
            self.rate = (0.0, 0.0)          # gap: start over
            return
        a = dt / (self.p.d_tau + dt)
        rf, rr = (f - prev[1]) / dt, (r - prev[2]) / dt
        self.rate = (self.rate[0] + a * (rf - self.rate[0]), self.rate[1] + a * (rr - self.rate[1]))

    def _horizontal(self, f, r):
        e = math.hypot(f, r)
        kp = 0.0 if e < self.p.deadband else self.p.kp
        vf = kp * f + self.p.kd * self.rate[0]
        vr = kp * r + self.p.kd * self.rate[1]
        if math.hypot(vf, vr) < 0.005:
            return 0.0, 0.0
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
            self._prev, self.rate = None, (0.0, 0.0)
            lost = None if self.last_valid_t is None else now - self.last_valid_t
            out["lost_s"] = None if lost is None else round(lost, 2)
            if self.committed and self.phase in ("FINAL", "TOUCHDOWN"):
                if h_m is None or h_m < p.commit_h + 0.10:
                    out["v_down"] = p.final_speed
                    out["note"] = "committed: blind descent"
                    if self.phase == "FINAL" and h_m is not None and h_m < p.touchdown_h:
                        self._set("TOUCHDOWN", now, h_m)
                    out["phase"] = self.phase
                    return self._round(out)
                self.committed = False          # lifted well above: normal rules again
            if self.phase in ("DESCEND", "FINAL") and lost is not None and lost >= p.lost_search_s:
                self.climb = True
            if lost is None or lost >= p.lost_search_s:
                self._set("SEARCH", now)            # also re-arms HOLD / TOUCHDOWN
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
        self._update_rate(f, r, now)

        if self.phase in ("HOLD", "TOUCHDOWN"):
            if self._lifted(h_m):
                self._set("SEARCH", now)
            else:
                out["phase"] = self.phase
                if self.phase == "TOUCHDOWN":
                    out["v_down"] = p.final_speed
                    out["note"] = "touchdown: pushing down, waiting to disarm"
                else:
                    out["note"] = "align timeout: pilot"
                return self._round(out)
        if self.phase == "SEARCH":
            self._set("ALIGN", now)

        out["v_fwd"], out["v_right"] = self._horizontal(f, r)
        # TOUCHDOWN stage: only descend (and commit) with the error inside align_tol
        pause_err = min(p.pause_err, p.align_tol) if self.strict_descent else p.pause_err

        if self.phase == "ALIGN":
            if e < p.align_tol:
                self.aligned_since = self.aligned_since or now
                if now - self.aligned_since >= p.align_hold_s:
                    self._set("DESCEND", now)
            else:
                self.aligned_since = None
                if self.align_timeout_enabled and now - self.phase_t > p.align_timeout_s:
                    self._set("HOLD", now, h_m)
                    out["v_fwd"] = out["v_right"] = 0.0
        if self.phase == "DESCEND":
            out["v_down"] = p.descend_speed if e <= pause_err else 0.0
            if h_m < p.final_h:
                self._set("FINAL", now)
        if self.phase == "FINAL":
            if h_m < p.commit_h and (not self.strict_descent or e <= pause_err):
                self.committed = True
            pause = e > pause_err and not self.committed
            out["v_down"] = 0.0 if pause else p.final_speed
            if h_m < p.touchdown_h and not pause:
                self._set("TOUCHDOWN", now, h_m)
                out["v_fwd"] = out["v_right"] = 0.0
                out["v_down"] = p.final_speed
        paused = e > pause_err and (self.phase == "DESCEND" or (self.phase == "FINAL" and not self.committed))
        if paused:
            out["note"] = "error > pause: holding height"
        out["phase"] = self.phase
        return self._round(out)

    @staticmethod
    def _round(out):
        for k in ("v_fwd", "v_right", "v_down", "err_m", "h_m"):
            if out.get(k) is not None:
                out[k] = round(float(out[k]), 4)
        return out
