"""GUIDED velocity sender: sends the landing law output to the Pixhawk, with interlocks.

The law (landing_law.py) runs every camera frame in camera_node and publishes its output
in VisionState.landing_shadow. This sender reads it at a fixed rate and sends it as a
BODY-frame velocity setpoint (forward / right / down) ONLY when all of these hold:

    1. the pilot started the mission and selected a stage (panel button or `landshadow`), which
       is refused unless the drone is armed, flying and already in GUIDED;
    2. the Pixhawk is armed and its last HEARTBEAT (recent) says GUIDED;
    3. the law output is fresh (camera thread alive).

It NEVER changes the flight mode. Leaving GUIDED (pilot switch, LAND on channel 7, failsafe,
disarm or lost heartbeat) disarms the sender: a stage must be selected again for the next GUIDED.
In GUIDED with a stale law output it sends zero velocity (brake). If the link drops, the
Pixhawk itself stops after ~3 s without setpoints.

Stages (what the law is allowed to do):
    align    horizontal only, v_down = 0 (first flight: check directions and gains)
    descend  (APPROACH) also descends, but never below approach_h (0.35-1.0 m, default floor_h):
             there it holds height, centred.
    land     the whole landing in GUIDED: no floor, it only descends with the xy error below
             5 cm (blind below the law's commit_h) and the motors are disarmed once it has really stopped on the ground
             (law in TOUCHDOWN, lidar below disarm_h and steady for disarm_after_s).
    stop     holds in GUIDED with zero velocity; the sticks do nothing until the pilot leaves
             GUIDED with channel 5.

Mission: START MISSION (mission_start) enables the stages and begins holding. STOP
(mission_stop) holds again and refuses every stage until the next START. Leaving GUIDED ends it.
"""

import math
import threading
import time
from dataclasses import dataclass, fields

GUIDED = 4                      # ArduCopter custom_mode
STAGES = ("align", "descend", "land", "stop")
APPROACH_MIN_M, APPROACH_MAX_M = 0.35, 1.0
STAGE_ALIASES = {"acercar": "descend", "approach": "descend", "touchdown": "land", "hold": "stop"}


@dataclass
class SenderParams:
    rate_hz: float = 10.0
    v_xy_max: float = 0.20      # m/s, on top of the law's own v_max
    v_down_max: float = 0.15    # m/s
    v_up_max: float = 0.15      # m/s
    law_max_age_s: float = 0.30
    hb_max_age_s: float = 1.5
    floor_h: float = 0.35       # descend stage: no descent below this lidar height, m
    min_flight_h: float = 0.15  # START and stages refused below this lidar height, m
    disarm_h: float = 0.14      # land stage: motors off only below this lidar height, m
    disarm_after_s: float = 1.5 # ... continuously for this long
    disarm_band_m: float = 0.03 # ... with the height moving less than this (not still descending)
    hold_kp: float = 0.6        # height hold with the lidar (stop/align/approach floor): m/s per m
    hold_v_max: float = 0.08    # ... at most this up or down, m/s
    hold_deadband: float = 0.02 # ... no correction inside this band, m
                                # (flight 2026-11-23: with v_down = 0 the drone sank 0.6 -> 0.15 m)

    @classmethod
    def from_dict(cls, d):
        names = {f.name for f in fields(cls)}
        return cls(**{k: float(v) for k, v in (d or {}).items() if k in names})


class GuidedSender:
    def __init__(self, mavlink, vision_state, params=None, log=None, start_thread=True):
        self.mav = mavlink              # MAVLinkManager: .state (VehicleState), .send_velocity_body
        self.vision_state = vision_state
        self.p = params or SenderParams()
        self._log = log or (lambda msg, level="INFO": print(f"[GUIDED_SENDER] {msg}"))
        self._lock = threading.Lock()
        self.stage = None               # None = disarmed (shadow only)
        self.mission = False            # START MISSION pressed: stages are allowed
        self.approach_h = self.p.floor_h  # APPROACH target height, m
        self._td_since = None           # touchdown confirmation window
        self._td_min = self._td_max = 0.0
        self._last_disarm_tx = None
        self._hold_h, self._hold_stage = None, None   # height latched for stop/align
        self.in_guided = False
        self.last = {"sent": False, "v_fwd": 0.0, "v_right": 0.0, "v_down": 0.0,
                     "reason": "disarmed", "t": None}
        self.sent_count = 0
        self.on_activate = None         # called when GUIDED starts (resets the law: its timers
        self._running = False           # must not run out while the pilot is still in LOITER)
        if start_thread:
            self.start()

    # ------------------------------------------------------------- control
    def start(self):
        if self._running:
            return
        self._running = True
        threading.Thread(target=self._loop, name="GuidedSender", daemon=True).start()

    def stop(self):
        self._running = False

    def arm(self, stage):
        stage = (stage or "").lower()
        stage = STAGE_ALIASES.get(stage, stage)
        if stage not in STAGES:
            return False, f"stage must be one of {STAGES}"
        guided, _why = self._guided_now()
        with self._lock:
            self.stage = stage
            if not guided:
                self.last.update(sent=False, reason="armed: waiting for GUIDED")
        if guided:
            return True, f"{stage} active in GUIDED. Leave GUIDED with ch5 to take control."
        return True, f"armed ({stage}). Flip ch5 to GUIDED to start; leaving GUIDED disarms it."

    def flight_check(self):
        """(ok, reason): the Pixhawk is linked, armed and in GUIDED, and the drone is in the air."""
        st = getattr(self.mav, "state", None)
        if st is None or not hasattr(st, "get_packet_with_time"):
            return False, "no MAVLink state"
        hb, hb_t = st.get_packet_with_time("HEARTBEAT")
        if not hb or hb_t is None or time.time() - hb_t > self.p.hb_max_age_s:
            return False, "no Pixhawk heartbeat"
        if not (int(hb.get("base_mode", 0)) & 128):
            return False, "drone is not armed"
        if int(hb.get("custom_mode", -1)) != GUIDED:
            return False, "not in GUIDED (flip ch5)"
        ls = self.vision_state.get_landing_shadow() if self.vision_state else {}
        t, h = ls.get("t"), ls.get("h_m")
        if t is None or time.monotonic() - t > self.p.law_max_age_s:
            return False, "no camera/landing law data"
        if h is None:
            return False, "no lidar height"
        if h < self.p.min_flight_h:
            return False, f"drone is not flying (lidar {h * 100:.0f} cm)"
        return True, ""

    def mission_start(self):
        ok, why = self.flight_check()
        if not ok:
            return False, why
        with self._lock:
            self.mission = True
        return self.arm("stop")

    def mission_stop(self):
        """Hold (in GUIDED) or cancel (outside it); every stage is refused until the next START."""
        if self._guided_now()[0]:
            self.arm("stop")
            with self._lock:
                self.mission = False
        else:
            self.disarm("pilot")

    def select(self, stage, approach_h=None):
        """A stage button: only works while the mission is started and the drone is flying."""
        if not self.mission:
            return False, "mission not started (press START MISSION)"
        name = STAGE_ALIASES.get((stage or "").lower(), (stage or "").lower())
        if name != "stop" and self.stage not in ("align", "descend", "land"):  # never block a step back
            ok, why = self.flight_check()
            if not ok:
                return False, why
        if name == "descend":
            target = self.p.floor_h if approach_h is None else float(approach_h)
            if not APPROACH_MIN_M <= target <= APPROACH_MAX_M:
                return False, f"approach height must be {APPROACH_MIN_M:.2f}-{APPROACH_MAX_M:.2f} m"
            self.approach_h = target
        return self.arm(stage)

    def disarm(self, reason="pilot"):
        with self._lock:
            was = self.stage
            self.stage = None
            self.mission = False
            self.last.update(sent=False, v_fwd=0.0, v_right=0.0, v_down=0.0, reason=f"disarmed: {reason}")
        if was and reason == "pilot" and self._guided_now()[0]:
            self.mav.send_velocity_body(0.0, 0.0, 0.0)  # do not coast on the last setpoint
        if was:
            self._log(f"sender disarmed ({reason}); nothing is sent", "WARN")

    def status(self):
        with self._lock:
            d = dict(self.last)
            d.update(stage=self.stage, in_guided=self.in_guided, sent_count=self.sent_count)
        return d

    def mode_word(self):
        """For the law output / logger: SHADOW, ARMED-<stage> (waiting) or ACTIVE-<stage>."""
        with self._lock:
            if not self.stage:
                return "SHADOW"
            return f"{'ACTIVE' if self.in_guided else 'ARMED'}-{self.stage}"

    # ------------------------------------------------------------- loop
    def _loop(self):
        period = 1.0 / max(1.0, self.p.rate_hz)
        while self._running:
            t0 = time.monotonic()
            try:
                self.tick()
            except Exception as e:  # noqa: BLE001 - never let the sender thread die silently
                self.disarm(f"error {e}")
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _guided_now(self):
        st = getattr(self.mav, "state", None)
        if st is None or not hasattr(st, "get_packet_with_time"):
            return False, "no MAVLink state"
        hb, hb_t = st.get_packet_with_time("HEARTBEAT")
        if not hb or hb_t is None:
            return False, "no heartbeat"
        if time.time() - hb_t > self.p.hb_max_age_s:
            return False, "heartbeat lost"
        if not (int(hb.get("base_mode", 0)) & 128):
            return False, "Pixhawk disarmed"
        if int(hb.get("custom_mode", -1)) != GUIDED:
            return False, "not GUIDED"
        return True, ""

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        guided, why = self._guided_now()
        with self._lock:
            stage, was = self.stage, self.in_guided
            self.in_guided = guided
        if stage and was and not guided:
            self.disarm(f"left GUIDED ({why})")
            return None
        if stage and guided and not was:
            self._log(f"GUIDED detected: sending law output ({stage})", "WARN")
            if self.on_activate:
                self.on_activate()
        if not stage or not guided:
            with self._lock:
                self.last.update(sent=False, reason="disarmed" if not stage else f"waiting: {why}")
            return None

        ls = self.vision_state.get_landing_shadow() if self.vision_state else {}
        t = ls.get("t")
        reason = "law"
        if stage == "stop":
            vf = vr = vd = 0.0
            reason = "stop: holding"
        elif t is None or now - t > self.p.law_max_age_s:
            vf = vr = vd = 0.0
            reason = "law stale: brake"
        else:
            vf, vr, vd = (float(ls.get(k) or 0.0) for k in ("v_fwd", "v_right", "v_down"))
            n = math.hypot(vf, vr)
            if n > self.p.v_xy_max:
                vf, vr = vf * self.p.v_xy_max / n, vr * self.p.v_xy_max / n
            vd = max(-self.p.v_up_max, min(self.p.v_down_max, vd))
            if stage == "align":
                vd = self._hold(stage, ls.get("h_m"))
            elif stage == "descend":
                h = ls.get("h_m")
                if h is None:
                    vd = min(vd, 0.0)
                elif h < self.approach_h + self.p.hold_deadband and vd >= 0:
                    vd = self._hold(stage, h, self.approach_h)
                    reason = "approach height: holding"
                elif vd > 0:
                    # brake before the floor (same gain as the hold) instead of arriving at
                    # full descend speed: the last flight undershot 0.60 m down to 0.49 m
                    cap = self.p.hold_kp * (h - self.approach_h)
                    if cap < vd:
                        vd = max(0.0, cap)
                        reason = "approach height: slowing"
        if stage == "stop" and t is not None and now - t <= self.p.law_max_age_s:
            vd = self._hold(stage, ls.get("h_m"))
        if stage not in ("stop", "align"):
            self._hold_stage = None
        self.mav.send_velocity_body(vf, vr, vd)
        self._check_touchdown(stage, ls, reason == "law", now)
        with self._lock:
            self.sent_count += 1
            self.last = {"sent": True, "v_fwd": round(vf, 4), "v_right": round(vr, 4),
                         "v_down": round(vd, 4), "reason": reason, "t": now}
        return vf, vr, vd

    def _hold(self, stage, h, target=None):
        """v_down that keeps the lidar height at target (stop/align: the height when the stage
        started). The Pixhawk alone, with v_down = 0, let the drone sink near the ground."""
        if h is None:
            return 0.0
        if target is None:
            if self._hold_stage != stage or self._hold_h is None:
                self._hold_stage, self._hold_h = stage, float(h)
            target = self._hold_h
        err = float(h) - target
        if abs(err) < self.p.hold_deadband:
            return 0.0
        return max(-self.p.hold_v_max, min(self.p.hold_v_max, self.p.hold_kp * err))

    def _check_touchdown(self, stage, ls, fresh, now):
        """Motors off (normal, not forced disarm) only when really stopped on the ground."""
        h = ls.get("h_m")
        if not (fresh and stage == "land" and ls.get("phase") == "TOUCHDOWN"
                and h is not None and h < self.p.disarm_h):
            self._td_since = None
            return
        if self._td_since is None or max(self._td_max, h) - min(self._td_min, h) > self.p.disarm_band_m:
            self._td_since, self._td_min, self._td_max = now, h, h
            return
        self._td_min, self._td_max = min(self._td_min, h), max(self._td_max, h)
        if now - self._td_since < self.p.disarm_after_s:
            return
        if self._last_disarm_tx is None or now - self._last_disarm_tx >= 1.0:
            self._last_disarm_tx = now
            self._log(f"touchdown confirmed at h={h:.2f} m: disarming motors", "WARN")
            self.mav.disarm()
