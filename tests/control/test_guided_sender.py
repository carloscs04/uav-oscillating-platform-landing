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


def test_arm_refused_while_in_guided():
    mav, vis, s = make()
    mav.state.set(GUIDED)
    assert not s.arm("align")[0]
    assert not s.arm("bogus")[0]


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
    vis.set(0.0, 0.0, 0.10, h=s.p.floor_h - 0.05)   # below the floor: no more descent
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


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("OK", name)
