"""GUIDED velocity sender: sends the landing law output to the Pixhawk, with interlocks.

The law (landing_law.py) runs every camera frame in camera_node and publishes its output
in VisionState.landing_shadow. This sender reads it at a fixed rate and sends it as a
BODY-frame velocity setpoint (forward / right / down) ONLY when all of these hold:

    1. the pilot armed the sender in the CLI (`landshadow arm align|descend`), while NOT
       in GUIDED: the drone only starts moving when the pilot flips channel 5 to GUIDED;
    2. the Pixhawk is armed and its last HEARTBEAT (recent) says GUIDED;
    3. the law output is fresh (camera thread alive).

It NEVER changes the flight mode. Leaving GUIDED (pilot switch, LAND on channel 7, failsafe,
disarm or lost heartbeat) disarms the sender: it must be armed again before the next GUIDED.
In GUIDED with a stale law output it sends zero velocity (brake). If the link drops, the
Pixhawk itself stops after ~3 s without setpoints.

Stages (what the law is allowed to do):
    align    horizontal only, v_down = 0 (first flight: check directions and gains)
    descend  also descends, but never below floor_h: there it holds height, centred, and the
             pilot lands with channel 7 (LAND). Final touchdown method still to be decided.
"""

import math
import threading
import time
from dataclasses import dataclass, fields

GUIDED = 4                      # ArduCopter custom_mode
STAGES = ("align", "descend")


@dataclass
class SenderParams:
    rate_hz: float = 10.0
    v_xy_max: float = 0.20      # m/s, on top of the law's own v_max
    v_down_max: float = 0.15    # m/s
    v_up_max: float = 0.15      # m/s
    law_max_age_s: float = 0.30
    hb_max_age_s: float = 1.5
    floor_h: float = 0.35       # descend stage: no descent below this lidar height, m

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
        self.in_guided = False
        self.last = {"sent": False, "v_fwd": 0.0, "v_right": 0.0, "v_down": 0.0,
                     "reason": "disarmed", "t": None}
        self.sent_count = 0
        self._running = False
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
        if stage not in STAGES:
            return False, f"stage must be one of {STAGES}"
        guided, why = self._guided_now()
        if guided:
            return False, "already in GUIDED: switch ch5 out of GUIDED, arm, then switch to GUIDED"
        with self._lock:
            self.stage = stage
            self.last.update(sent=False, reason="armed: waiting for GUIDED")
        return True, f"armed ({stage}). Flip ch5 to GUIDED to start; leaving GUIDED disarms it."

    def disarm(self, reason="pilot"):
        with self._lock:
            was = self.stage
            self.stage = None
            self.last.update(sent=False, v_fwd=0.0, v_right=0.0, v_down=0.0, reason=f"disarmed: {reason}")
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
        if not stage or not guided:
            with self._lock:
                self.last.update(sent=False, reason="disarmed" if not stage else f"waiting: {why}")
            return None

        ls = self.vision_state.get_landing_shadow() if self.vision_state else {}
        t = ls.get("t")
        reason = "law"
        if t is None or now - t > self.p.law_max_age_s:
            vf = vr = vd = 0.0
            reason = "law stale: brake"
        else:
            vf, vr, vd = (float(ls.get(k) or 0.0) for k in ("v_fwd", "v_right", "v_down"))
            n = math.hypot(vf, vr)
            if n > self.p.v_xy_max:
                vf, vr = vf * self.p.v_xy_max / n, vr * self.p.v_xy_max / n
            vd = max(-self.p.v_up_max, min(self.p.v_down_max, vd))
            if stage == "align":
                vd = 0.0
            elif stage == "descend":
                h = ls.get("h_m")
                if (h is None or h < self.p.floor_h) and vd > 0:
                    vd = 0.0
                    reason = "floor: pilot LAND (ch7)"
        self.mav.send_velocity_body(vf, vr, vd)
        with self._lock:
            self.sent_count += 1
            self.last = {"sent": True, "v_fwd": round(vf, 4), "v_right": round(vr, 4),
                         "v_down": round(vd, 4), "reason": reason, "t": now}
        return vf, vr, vd
