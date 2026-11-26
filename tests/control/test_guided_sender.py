"""
test_guided_sender.py - GUIDED sender interlocks (no hardware).

    python3 tests/control/test_guided_sender.py
    python3 -m pytest tests/control/test_guided_sender.py
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.logic.control.guided_sender import GUIDED, GuidedSender  # noqa: E402

LOITER = 5


class FakeVehicle:
    def __init__(self):
        self.hb, self.hb_t = None, None

    def set(self, mode, armed=True, age=0.0):
        self.hb = {"custom_mode": mode, "base_mode": 128 if armed else 0}
        self.hb_t = time.time() - age

    def get_packet_with_time(self, name):
        return (self.hb, self.hb_t) if name == "HEARTBEAT" else (None, None)


class FakeMav:
    def __init__(self):
        self.state = FakeVehicle()
        self.sent = []

    def send_velocity_body(self, f, r, d):
        self.sent.append((f, r, d))

    def disarm(self):
        self.disarms = getattr(self, "disarms", 0) + 1


class FakeVision:
    def __init__(self):
        self.ls = {}

    def set(self, f, r, d, h=1.0, t=None):
        self.ls = {"v_fwd": f, "v_right": r, "v_down": d, "h_m": h,
                   "t": time.monotonic() if t is None else t}

    def get_landing_shadow(self):
        return dict(self.ls)


def make():
    mav, vis = FakeMav(), FakeVision()
    return mav, vis, GuidedSender(mav, vis, log=lambda *a, **k: None, start_thread=False)


def test_nothing_sent_unless_armed_and_guided():
    mav, vis, s = make()
    vis.set(0.1, 0.0, 0.1)
    mav.state.set(GUIDED)
    s.tick()                                  # GUIDED but sender disarmed
    mav.state.set(LOITER)
    assert s.arm("align")[0]
    s.tick()                                  # armed but LOITER
    assert mav.sent == []


def test_stage_selected_inside_guided_starts_at_once():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    s.tick()
    assert not s.arm("bogus")[0] and s.stage is None
    assert s.arm("acercar")[0] and s.stage == "descend"
    vis.set(0.05, 0.0, 0.10, h=1.0)
    assert s.tick() == (0.05, 0.0, 0.10)
    assert s.arm("touchdown")[0] and s.stage == "land"
    vis.set(0.0, 0.0, 0.10, h=0.20)           # below the acercar floor: touchdown keeps going
    assert s.tick()[2] == 0.10


def test_stop_holds_even_with_a_law_command():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    s.tick()
    s.arm("stop")
    vis.set(0.2, 0.2, 0.15)
    assert s.tick() == (0.0, 0.0, 0.0)
    mav.state.set(LOITER)
    s.tick()
    assert s.stage is None


def test_pilot_disarm_in_guided_sends_one_zero():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    s.tick()
    s.arm("align")
    s.disarm("pilot")
    assert mav.sent == [(0.0, 0.0, 0.0)]


def test_stages_work_alone_only_in_guided_and_flying():
    mav, vis, s = make()
    vis.set(0.0, 0.0, 0.0, h=0.05)
    mav.state.set(GUIDED)
    assert not s.select("align")[0] and s.stage is None        # on the ground
    assert not s.mission_start()[0] and not s.mission
    vis.set(0.0, 0.0, 0.0, h=1.0)
    mav.state.set(LOITER)
    assert "GUIDED" in s.select("align")[1] and s.stage is None
    assert "GUIDED" in s.mission_start()[1] and not s.mission
    mav.state.set(GUIDED, armed=False)
    assert "not armed" in s.select("touchdown")[1]
    mav.state.set(GUIDED)
    assert s.select("align")[0] and s.stage == "align" and not s.mission   # no START needed
    assert not s.select("approach", 1.5)[0] and s.stage == "align"
    assert s.select("approach", 0.45)[0] and s.approach_h == 0.45
    vis.set(0.0, 0.0, 0.1, h=0.40)
    assert -0.08 <= s.tick()[2] < 0.0                          # below the typed height: climbs back
    assert s.select("approach")[0] and s.approach_h == 0.35
    assert s.select("touchdown")[0] and s.stage == "land"
    assert s.mission_start()[0] and s.mission and s.stage == "align"
    assert s.mission_stop() == "align" and not s.mission          # stopped in ALIGN: stays
    assert s.select("touchdown")[0]
    mav.state.set(LOITER)
    s.tick()
    assert s.stage is None and not s.mission


def test_start_mission_runs_the_whole_sequence():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    p = s.p
    t = [time.monotonic()]

    def law(phase, h, err=0.05, vd=0.1):
        vis.ls = {"v_fwd": 0.0, "v_right": 0.0, "v_down": vd, "h_m": h, "t": t[0],
                  "phase": phase, "err_m": err}

    def run(seconds, *a, **k):
        end = t[0] + seconds
        while t[0] < end:
            law(*a, **k)
            s.tick(now=t[0])
            t[0] += 0.1

    law("ALIGN", 1.0)
    assert s.mission_start()[0] and s.stage == "align" and s.auto_step == "align"
    run(2.0, "ALIGN", 1.0)
    assert s.auto_step == "align" and mav.sent[-1][2] == 0.0  # aligning: no descent
    run(0.2, "DESCEND", 1.0)
    assert s.auto_step == "approach1" and s.stage == "descend" and s.approach_h == p.auto_h1
    run(p.auto_hold1_s - 1.0, "DESCEND", p.auto_h1)
    assert s.auto_step == "approach1"                        # still waiting the 25 s
    run(1.5, "DESCEND", p.auto_h1)
    assert s.auto_step == "approach2" and s.approach_h == p.auto_h2
    run(p.auto_hold2_s + 0.5, "DESCEND", p.auto_h2)
    assert s.auto_step == "centring" and s.stage == "descend"
    run(3.0, "DESCEND", p.auto_h2, err=0.04)                 # 4 cm: never touches down
    assert s.stage == "descend"
    run(0.3, "DESCEND", p.auto_h2, err=0.02)
    run(0.1, "DESCEND", p.auto_h2, err=0.05)                 # broken: timer restarts
    run(0.8, "DESCEND", p.auto_h2, err=0.02)
    assert s.auto_step == "touchdown" and s.stage == "land" and s.mission


def test_stage_button_ends_the_sequence():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    vis.set(0.0, 0.0, 0.0, h=1.0)
    s.mission_start()
    assert s.select("approach", 0.5)[0] and not s.mission and s.auto_step is None


def test_stop_and_align_hold_the_lidar_height():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    s.tick()
    vis.set(0.0, 0.0, 0.0, h=0.60)
    s.arm("stop")
    assert s.tick()[2] == 0.0                     # latches 0.60
    vis.set(0.0, 0.0, 0.0, h=0.50)                # sinking with v_down = 0 (flight 2026-11-23)
    assert -s.p.hold_v_max <= s.tick()[2] < 0.0   # climbs
    vis.set(0.0, 0.0, 0.0, h=0.70)
    assert 0.0 < s.tick()[2] <= s.p.hold_v_max    # comes down
    s.arm("align")
    vis.set(0.02, 0.0, 0.0, h=0.70)
    assert s.tick()[2] == 0.0                     # new stage: latches its own height


def _land_tick(mav, vis, s, t, h, phase="TOUCHDOWN"):
    vis.set(0.0, 0.0, 0.05, h=h, t=t)
    vis.ls["phase"] = phase
    s.tick(now=t)
    return getattr(mav, "disarms", 0)


def test_can_step_back_from_touchdown_when_low():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    s.tick()
    vis.set(0.0, 0.0, 0.0, h=1.0)
    s.mission_start()
    s.select("touchdown")
    vis.set(0.0, 0.0, 0.1, h=0.12)                # below min_flight_h
    assert s.select("approach")[0] and s.stage == "descend"
    assert s.tick()[2] == -s.p.hold_v_max         # climbs back to the approach height


def test_motors_off_only_when_stopped_on_ground():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    s.tick()
    vis.set(0.0, 0.0, 0.0, h=1.0)
    s.mission_start()
    s.select("touchdown")
    assert _land_tick(mav, vis, s, 100.0, 0.05, phase="FINAL") == 0     # low but not TOUCHDOWN
    for i in range(5):                                  # still descending at final speed (0.1 m/s)
        assert _land_tick(mav, vis, s, 101.0 + i * 0.2, 0.135 - i * 0.02) == 0
    assert _land_tick(mav, vis, s, 110.0, 0.05, phase="FINAL") == 0
    assert _land_tick(mav, vis, s, 111.0, 0.30) == 0                    # lidar jump restarts
    assert _land_tick(mav, vis, s, 112.0, 0.05) == 0
    assert _land_tick(mav, vis, s, 112.3, 0.08) == 0                    # settling 3 cm: still counts
    assert _land_tick(mav, vis, s, 112.7, 0.07) == 1                    # 0.7 s on the ground


def test_align_sends_horizontal_only_and_leaving_guided_disarms():
    mav, vis, s = make()
    mav.state.set(LOITER)
    s.arm("align")
    mav.state.set(GUIDED)
    vis.set(0.05, -0.04, 0.15)
    s.tick()
    assert mav.sent[-1] == (0.05, -0.04, 0.0)
    mav.state.set(LOITER)                     # pilot switches out (or LAND on ch7)
    s.tick()
    assert s.stage is None
    mav.state.set(GUIDED)                     # back to GUIDED: must NOT resume
    s.tick()
    assert len(mav.sent) == 1


def test_limits_stale_and_floor():
    mav, vis, s = make()
    mav.state.set(LOITER)
    s.arm("descend")
    mav.state.set(GUIDED)
    vis.set(1.0, 1.0, 1.0, h=1.0)             # way over the limits
    f, r, d = s.tick()
    assert abs((f * f + r * r) ** 0.5 - s.p.v_xy_max) < 1e-6 and d == s.p.v_down_max
    vis.set(0.0, 0.0, 0.10, h=s.p.floor_h - 0.05)   # below the floor: climbs back slowly
    assert -s.p.hold_v_max <= s.tick()[2] < 0.0
    vis.set(0.0, 0.0, 0.10, h=s.p.floor_h + 0.01)   # at the floor: holds
    assert s.tick()[2] == 0.0
    vis.set(0.0, 0.0, -0.10, h=0.2)           # climbing is still allowed
    assert s.tick()[2] == -0.10
    vis.set(0.1, 0.1, 0.1, t=time.monotonic() - 1.0)  # camera thread dead -> brake
    assert s.tick() == (0.0, 0.0, 0.0)


def test_heartbeat_lost_or_disarmed_stops():
    for kw in ({"age": 5.0}, {"armed": False}):
        mav, vis, s = make()
        mav.state.set(LOITER)
        s.arm("align")
        mav.state.set(GUIDED)
        vis.set(0.05, 0.0, 0.0)
        s.tick()
        mav.state.set(GUIDED, **kw)
        s.tick()
        assert s.stage is None and len(mav.sent) == 1


def test_entering_guided_resets_law_once():
    mav, vis, s = make()
    calls = []
    s.on_activate = lambda: calls.append(1)
    mav.state.set(LOITER)
    s.arm("align")
    s.tick()
    assert calls == []
    mav.state.set(GUIDED)
    vis.set(0.0, 0.0, 0.0)
    s.tick()
    s.tick()
    assert calls == [1]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("OK", name)


def test_descend_brakes_before_the_approach_floor():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    s.tick()
    vis.set(0.0, 0.0, 0.0, h=1.0)
    s.mission_start()
    assert s.select("approach", 0.6)[0]
    vis.set(0.0, 0.0, 0.15, h=1.0)               # far above: full law speed
    assert abs(s.tick()[2] - 0.15) < 1e-9
    vis.set(0.0, 0.0, 0.15, h=0.7)               # 10 cm above: slowed to kp * 0.10
    assert abs(s.tick()[2] - s.p.hold_kp * 0.10) < 1e-6
    assert s.last["reason"] == "approach height: slowing"


def test_land_wait_holds_lidar_height():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    s.tick()
    vis.set(0.0, 0.0, 0.0, h=0.25)
    s.mission_start()
    s.select("touchdown")
    vis.ls["phase"], vis.ls["err_m"] = "FINAL", 0.06
    s.tick()                                      # latches 0.25 m
    assert s.last["reason"] == "waiting for error: holding"
    vis.set(0.0, 0.0, 0.0, h=0.20)
    vis.ls["phase"], vis.ls["err_m"] = "FINAL", 0.06
    assert s.tick()[2] < 0                        # sank 5 cm while waiting -> climbs back


def test_land_wait_without_marker_never_descends():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    s.tick()
    vis.set(0.0, 0.0, 0.0, h=0.25)
    s.mission_start()
    s.select("touchdown")
    vis.ls["phase"], vis.ls["err_m"] = "FINAL", 0.06
    s.tick()                                      # latches 0.25 m
    vis.set(0.0, 0.0, 0.0, h=0.45)                # climbed to look again, marker not seen
    vis.ls["phase"], vis.ls["err_m"] = "FINAL", None
    assert s.tick()[2] <= 0                       # 20:09:11: pulled back down 0.15 m/s blind
    vis.set(0.0, 0.0, 0.0, h=0.45)
    vis.ls["phase"], vis.ls["err_m"] = "FINAL", 0.06
    assert s.tick()[2] == 0.0                     # marker back: holds 0.45, not the old 0.25


def test_stop_mission_keeps_the_stage_touchdown_backs_off_to_35():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    vis.set(0.0, 0.0, 0.0, h=1.0)
    s.mission_start()
    s.arm("descend"); s.approach_h = 0.70; s.mission = True
    assert s.mission_stop() == "descend" and s.approach_h == 0.70 and not s.mission
    s.mission_start()
    s.arm("land"); s.mission = True
    assert s.mission_stop() == "descend" and s.approach_h == s.p.floor_h
    vis.set(0.0, 0.0, 0.1, h=0.15)
    assert s.tick()[2] < 0.0                                  # climbs back to 35 cm
    mav.state.set(LOITER)
    assert s.mission_stop() is None and s.stage is None
