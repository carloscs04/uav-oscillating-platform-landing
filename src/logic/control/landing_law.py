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
    kp_final: float = 0.3           # FINAL: stronger pull to the centre (flight 2026-11-25 19:10: with kp 0.35
                                    # a 3-4 cm error asked only 1-2 cm/s and drifted 0.4 -> 4.4 cm in 1 s)
    kd_final: float = 0.6           # FINAL (below final_h): damping ...
    v_max_final: float = 0.08       # ... and sideways limit (0.06 until 2026-11-25 21:50: could not brake an
                                    # 8 cm/s drift, 2 -> 8 cm in 1.5 s at 0.30 m; kd 0.6 now, not 0.3 as 19:38) (flight 2026-11-24 22:03: at 0.2-0.3 m kd 0.9
                                    # with v_max 0.3 commanded 0.15-0.28 m/s, swung up to 16 cm;
                                    # 2026-11-25 19:38: kd 0.3 / 0.08 m/s could not stop an 8 cm/s
                                    # drift, overshot to 12 cm and lost the marker 4 times;
                                    # 2026-11-25 20:08: kp 0.5 / 0.12 m/s swung +-10 cm with a 2.5 s
                                    # period (drone answers ~0.5 s late), now kp 0.3 / 0.06 m/s)
    settle_rate: float = 0.05       # (0.03 until 2026-11-25 22:00: the drone almost never moves slower, it
                                    # waited 22 s in 0.30-0.40 m; FINAL still has to centre 0.5 s to commit)
                                    # DESCEND: below final_h + slow_band descend only while the error
                                    # moves slower than this (enter FINAL still, not passing through), m/s
    reacquire_dh: float = 0.10      # DESCEND/FINAL lost below final_h + this: after lost_hold_s climb at
    reacquire_speed: float = 0.10   # this until it is seen again (camera sees only +-7 cm at 0.28 m), m/s
    deadband: float = 0.02          # error below this -> no horizontal command, m
    deadband_final: float = 0.008   # FINAL/TOUCHDOWN: P acts from here (2026-11-25 20:28: with the 2 cm
                                    # deadband the error grew 0.3 -> 3.4 cm unchecked while descending
                                    # 0.29 -> 0.18 m, lost the marker and finished blind)
    align_tol: float = 0.05         # "aligned" when |err| below this, m
    align_hold_s: float = 1.0       # ...for this long before descending (2.0 until 2026-11-25 22:12: inside
                                    # 5 cm for 1.85 s at 1.2 m and started over; 17 s in ALIGN, battery low)
    align_timeout_s: float = 30.0   # ALIGN longer than this -> HOLD
    descend_speed: float = 0.15     # m/s
    pause_err: float = 0.10         # stop descending while |err| above this, m
    final_h: float = 0.45           # FINAL below this height, m (0.30 until 2026-11-25 22:18: hovering at
                                    # 0.30-0.40 m the Pixhawk drifted 15-20 cm/s on its own and lost the marker)
    final_speed: float = 0.10       # m/s (0.07 until 2026-11-25 22:18: less time hovering near the ground)
    slow_band: float = 0.10         # TOUCHDOWN stage: already final_speed this far above final_h
                                    # (2026-11-25 19:22: arrived at 0.30 m at 15 cm/s, sank to 0.17)
    final_pause_err: float = 0.03   # (0.025 until 2026-11-25 22:24: the Pixhawk wobbles +-4 cm with a ~2.5 s
                                    # period on its own, so the predicted error almost never stayed under it)
                                    # (0.03 until 2026-11-25 20:28: drifted to 3.4 cm before stopping) FINAL (even committed): wait, no descent, while |err| above this, m
                                    # (was 0.05: on 2026-11-25 it reached 4.4 cm without ever pausing)
                                    # (flight 2026-11-24: committed at 5 cm, then blind-ish descent to 7 cm off)
    final_pause_min_h: float = 0.20 # ...but below this height keep descending if within blind_err, m
                                    # (0.15 until 2026-11-25 22:00: paused at 0.17 m with 2.6 cm, drifted to
                                    # 5.9 cm in 0.3 s in ground effect, lost the marker; better to finish)
    commit_h: float = 0.50          # commit (centred and still) below this, m (0.45 until 2026-11-25 23:02:
                                    # FINAL bobbed 0.38-0.50 m and the drone was centred within 1 cm for
                                    # ~1 s at 0.46-0.47 m without committing; 0.50 = the whole FINAL band)
                                    # (flight 2026-11-22: the marker left the frame at 0.22 m)
    commit_err: float = 0.03        # (0.025 until 2026-11-25 22:24, see final_pause_err)
                                    # TOUCHDOWN stage: commit only centred within this ...
    commit_hold_s: float = 0.2      # (0.5 until 2026-11-25 22:24: 0.5 s inside 2.5 cm happened once in 100 s)
                                    # ... for this long (flight 2026-11-24 21:53: committed at 1.9 cm,
                                    # drifted to 4.3 cm, lost the marker at 0.22 m and went down blind)
    blind_h: float = 0.20           # committed and marker lost: blind descent only below this, m
    abort_err: float = 0.05         # land stage FINAL, committed: error above this -> climb above final_h and
                                    # retry. Uncommitted it only holds height and centres (2026-11-25 22:24:
                                    # climbing on every 5 cm swing at 0.45 m looped up/down for 100 s)
    final_hold_max_s: float = 1.5   # committed below hold_retry_h, holding longer than this -> climb, retry
    hold_retry_h: float = 0.35      # above this a committed hold just waits (hovering 0.35-0.45 m was fine)
                                    # (2026-11-25 22:18: each hold low ended drifting off and losing it)
    lead_s: float = 0.7             # FINAL commit / resume use the error predicted this far ahead
                                    # (2026-11-25 22:12: committed at 2.1 cm sliding 4 cm/s -> 5.4 cm; resumed
                                    # at 2.0 cm closing at 7 cm/s -> overshot to 6.9 cm, lost at 0.19 m;
                                    # the Pixhawk followed the commands ~1.0 s late at ~0.5x)
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
        self.retry = False          # FINAL: climbing back above final_h to try again
        self.hold_since = None      # committed FINAL: holding height since
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
            self.retry, self.hold_since = False, None

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
        db = self.p.deadband_final if low else self.p.deadband
        kp = 0.0 if e < db else (self.p.kp_final if low else self.p.kp)
        kd = self.p.kd_final if low else self.p.kd
        vmax = self.p.v_max_final if low else self.p.v_max
        vf = kp * f + kd * self.rate[0]
        vr = kp * r + kd * self.rate[1]
        if math.hypot(vf, vr) < (0.002 if low else 0.005):
            return 0.0, 0.0
        n = math.hypot(vf, vr)
        if n > vmax:
            vf, vr = vf * vmax / n, vr * vmax / n
        return vf, vr

    def step(self, target, h_m, now):
        p = self.p
        self._low = False
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
                if h_m is None or h_m < p.blind_h:
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
            elif (self.phase in ("DESCEND", "FINAL") and lost is not None and lost >= p.lost_hold_s
                    and h_m is not None and h_m < p.final_h + p.reacquire_dh):
                out.update(phase=self.phase, v_down=-p.reacquire_speed,
                           note="lost low: climbing to see it again")
                return self._round(out)
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

        # calm FINAL gains already in the slow band above final_h (2026-11-25 21:50: at 0.30-0.40 m
        # the DESCEND gains asked up to 13 cm/s, swung +-8 cm with a ~2 s period and entered FINAL
        # still drifting 8 cm/s, which the 6 cm/s limit could not stop)
        low = self.phase in ("FINAL", "TOUCHDOWN") or (h_m is not None and h_m < p.final_h + p.slow_band)
        self._low = low
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
            speed = p.descend_speed
            if self.strict_descent and h_m is not None and h_m < p.final_h + p.slow_band:
                speed = p.final_speed
            moving = math.hypot(*self.rate)
            settling = h_m is not None and h_m < p.final_h + p.slow_band and moving > p.settle_rate
            out["v_down"] = speed if (e <= pause_err and not settling) else 0.0
            if settling and e <= pause_err:
                out["note"] = f"settling: moving {moving * 100:.1f} cm/s, holding height"
            if h_m < p.final_h:
                self._set("FINAL", now)
        if (self.phase == "FINAL" and self.strict_descent and not self.committed
                and h_m is not None and h_m >= p.final_h + 0.05):
            # lifted back above final_h (reacquire climb): FINAL never descends uncommitted and
            # cannot commit up here, so go back to DESCEND (2026-11-25 20:18: stuck at 0.40 m 50 s)
            self._set("DESCEND", now)
            out["v_down"] = 0.0
        if self.phase == "FINAL":
            # where the error will be lead_s from now at the current drift: the Pixhawk answers
            # ~1 s late, so "centred but sliding" must not count as centred
            e_lead = max(e, math.hypot(f + self.rate[0] * p.lead_s, r + self.rate[1] * p.lead_s))
            if self.strict_descent and not self.committed:
                self.centred_since = (self.centred_since or now) if e_lead <= p.commit_err else None
            why = None
            if self.strict_descent and self.committed and not self.retry:
                if e > p.abort_err:
                    why = f"error {e * 100:.1f} cm > {p.abort_err * 100:.0f} cm"
                elif self.committed and h_m < p.final_pause_min_h and e > p.blind_err:
                    why = f"error {e * 100:.1f} cm at {h_m:.2f} m"
                elif (h_m < p.hold_retry_h and self.hold_since is not None
                        and now - self.hold_since > p.final_hold_max_s):
                    why = f"held {now - self.hold_since:.1f} s low"
                if why:
                    self.retry, self.committed = True, False
                    self.centred_since = self.hold_since = None
            if self.retry:
                # never hover low off-centre: go back up where the Pixhawk and the camera work
                out["v_down"] = -p.reacquire_speed
                out["note"] = (why + ": " if why else "") + "climbing to retry"
        if self.phase == "FINAL" and not self.retry:
            if h_m < p.commit_h and not self.committed and (
                    not self.strict_descent
                    or (self.centred_since is not None and now - self.centred_since >= p.commit_hold_s)):
                self.committed = True
            pause = (e > pause_err and not self.committed) or \
                    (e_lead > p.final_pause_err and h_m >= p.final_pause_min_h) or \
                    (e > p.blind_err and h_m < p.final_pause_min_h) or \
                    (self.strict_descent and not self.committed)
            out["v_down"] = 0.0 if pause else p.final_speed
            self.hold_since = (self.hold_since or now) if (pause and self.committed) else None
            if h_m < p.touchdown_h and not pause:
                self._set("TOUCHDOWN", now, h_m)
                out["v_fwd"] = out["v_right"] = 0.0
                out["v_down"] = p.final_speed
        if self.phase == "DESCEND" and out["note"]:
            pass
        elif self.phase == "DESCEND" and e > pause_err:
            out["note"] = "error > pause: holding height"
        elif self.phase == "FINAL" and out["v_down"] == 0.0:
            if self.strict_descent and not self.committed and e <= pause_err:
                out["note"] = (f"centring: error {e * 100:.1f} cm, need <= {p.commit_err * 100:.1f} cm "
                               f"for {p.commit_hold_s:.1f} s")
            else:
                if e > pause_err and not self.committed:
                    out["note"] = f"waiting: error {e * 100:.1f} cm > {pause_err * 100:.1f} cm, not descending"
                else:
                    out["note"] = (f"waiting: error {e * 100:.1f} cm (in {p.lead_s:.1f} s {e_lead * 100:.1f} cm) "
                                   f"> {p.final_pause_err * 100:.1f} cm, not descending")
        out["phase"] = self.phase
        return self._round(out)

    def _round(self, out):
        # extra fields for the tuning log: error rate (D input), gain set in use, commit state
        out["rate_fwd"], out["rate_right"] = round(self.rate[0], 4), round(self.rate[1], 4)
        out["gains"] = "low" if getattr(self, "_low", False) else "high"
        out["committed"] = int(bool(self.committed))
        for k in ("v_fwd", "v_right", "v_down", "err_m", "h_m"):
            if out.get(k) is not None:
                out[k] = round(float(out[k]), 4)
        return out
