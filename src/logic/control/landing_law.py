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
    kp: float = 0.35                # m/s per m of error (0.35 -> 10 cm error = 3.5 cm/s)
                                    # (flight 2026-11-22: Pixhawk velocity lags 2-3 s, 0.5 swung;
                                    # 0.25 until 2026-11-24: wandered 3-8 cm for 12 s at 0.40 m, too weak)
    kd: float = 0.9                 # s: m/s per m/s of error rate (damps the drift; 0.4 then 0.6 until
                                    # 2026-11-24, raised with kp so the damping ratio grows)
    d_tau: float = 0.25             # low-pass time constant of the error rate, s
    v_max: float = 0.30             # horizontal speed limit, m/s
    kp_final: float = 0.5           # FINAL: stronger pull to the centre (flight 2026-11-25 19:10: with kp 0.35
                                    # a 3-4 cm error asked only 1-2 cm/s and drifted 0.4 -> 4.4 cm in 1 s)
    kd_final: float = 0.3           # FINAL (below final_h): softer damping ...
    v_max_final: float = 0.08       # ... and slower sideways (flight 2026-11-24 22:03: at 0.2-0.3 m kd 0.9
                                    # commanded 0.15-0.28 m/s for 3-5 cm errors, swung up to 16 cm
                                    # and lost the marker 3 times)
    deadband: float = 0.02          # error below this -> no horizontal command, m
    align_tol: float = 0.05         # "aligned" when |err| below this, m
    align_hold_s: float = 2.0       # ...for this long before descending
    align_timeout_s: float = 30.0   # ALIGN longer than this -> HOLD
    descend_speed: float = 0.15     # m/s
    pause_err: float = 0.10         # stop descending while |err| above this, m
    final_h: float = 0.30           # FINAL below this height, m
    final_speed: float = 0.07       # m/s (was 0.10: more time to correct per cm of descent)
    final_pause_err: float = 0.03   # FINAL (even committed): wait, no descent, while |err| above this, m
                                    # (was 0.05: on 2026-11-25 it reached 4.4 cm without ever pausing)
                                    # (flight 2026-11-24: committed at 5 cm, then blind-ish descent to 7 cm off)
    final_pause_min_h: float = 0.15 # ...but below this height keep descending (too close to hover), m
    commit_h: float = 0.30          # below this: no pauses, lost target -> keep descending, m
                                    # (flight 2026-11-22: the marker left the frame at 0.22 m)
    commit_err: float = 0.025       # TOUCHDOWN stage: commit only centred within this ...
    commit_hold_s: float = 0.5      # ... for this long (flight 2026-11-24 21:53: committed at 1.9 cm,
                                    # drifted to 4.3 cm, lost the marker at 0.22 m and went down blind)
    blind_err: float = 0.035        # TOUCHDOWN stage: marker lost low -> blind descent only if the last
                                    # error was below this; else brake, then climb and retry
    touchdown_h: float = 0.13       # TOUCHDOWN below this lidar height, m (on the ground it reads 0.09-0.10)
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
        self.centred_since = None   # strict commit: |err| <= commit_err since
        self.last_err = None        # last valid |err|, m
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
            self.centred_since = None

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

    def _horizontal(self, f, r, low=False):
        e = math.hypot(f, r)
        kp = 0.0 if e < self.p.deadband else (self.p.kp_final if low else self.p.kp)
        kd = self.p.kd_final if low else self.p.kd
        vmax = self.p.v_max_final if low else self.p.v_max
        vf = kp * f + kd * self.rate[0]
        vr = kp * r + kd * self.rate[1]
        if math.hypot(vf, vr) < 0.005:
            return 0.0, 0.0
        n = math.hypot(vf, vr)
        if n > vmax:
            vf, vr = vf * vmax / n, vr * vmax / n
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
            if (self.committed and self.strict_descent and self.phase == "FINAL"
                    and self.last_err is not None and self.last_err > p.blind_err):
                self.committed = False          # lost while off-centre: no blind descent
                out["note"] = f"lost {self.last_err * 100:.1f} cm off-centre: no blind descent"
            if self.committed and self.phase in ("FINAL", "TOUCHDOWN"):
                if h_m is None or h_m < p.commit_h + 0.10:
                    out["v_down"] = p.final_speed
                    out["note"] = "committed: blind descent"
                    if self.phase == "FINAL" and h_m is not None and h_m < p.touchdown_h:
                        self._set("TOUCHDOWN", now, h_m)
                    out["phase"] = self.phase
                    return self._round(out)
                self.committed = False          # lifted well above: normal rules again
            # Already on the ground (lidar at landed height) but the marker is gone: that is a landing,
            # not a loss. Flight 2026-11-24 22:27: lost #20 at 16 cm, sank onto the pad (lidar 0.09),
            # sat 2.5 s with motors up, then "lost low: climbing" lifted off the ground and it went
            # unstable. TOUCHDOWN lets the sender disarm once the height is steady.
            if (self.phase in ("FINAL", "TOUCHDOWN") and h_m is not None and h_m < p.touchdown_h):
                self._set("TOUCHDOWN", now, h_m)
                self.climb = False
                out.update(phase=self.phase, v_down=p.final_speed,
                           note="on the ground, marker lost: touchdown")
                return self._round(out)
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
        self.last_err = e
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

        low = self.phase in ("FINAL", "TOUCHDOWN") or (h_m is not None and h_m < p.final_h)
        out["v_fwd"], out["v_right"] = self._horizontal(f, r, low)
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
            if self.strict_descent and not self.committed:
                self.centred_since = (self.centred_since or now) if e <= p.commit_err else None
            if h_m < p.commit_h and not self.committed and (
                    not self.strict_descent
                    or (self.centred_since is not None and now - self.centred_since >= p.commit_hold_s)):
                self.committed = True
            pause = (e > pause_err and not self.committed) or \
                    (e > p.final_pause_err and h_m >= p.final_pause_min_h) or \
                    (self.strict_descent and not self.committed)
            out["v_down"] = 0.0 if pause else p.final_speed
            if h_m < p.touchdown_h and not pause:
                self._set("TOUCHDOWN", now, h_m)
                out["v_fwd"] = out["v_right"] = 0.0
                out["v_down"] = p.final_speed
        if self.phase == "DESCEND" and e > pause_err:
            out["note"] = "error > pause: holding height"
        elif self.phase == "FINAL" and out["v_down"] == 0.0:
            if self.strict_descent and not self.committed and e <= pause_err:
                out["note"] = (f"centring: error {e * 100:.1f} cm, need <= {p.commit_err * 100:.1f} cm "
                               f"for {p.commit_hold_s:.1f} s")
            else:
                lim = pause_err if (e > pause_err and not self.committed) else p.final_pause_err
                out["note"] = f"waiting: error {e * 100:.1f} cm > {lim * 100:.0f} cm, not descending"
        out["phase"] = self.phase
        return self._round(out)

    @staticmethod
    def _round(out):
        for k in ("v_fwd", "v_right", "v_down", "err_m", "h_m"):
            if out.get(k) is not None:
                out[k] = round(float(out[k]), 4)
        return out
