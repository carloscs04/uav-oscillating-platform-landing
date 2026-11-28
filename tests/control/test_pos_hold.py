"""
test_pos_hold.py - POSITION HOLD stage (Pi position targets) and PadLocator, no hardware.

    python3 tests/control/test_pos_hold.py
    python3 -m pytest tests/control/test_pos_hold.py
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.logic.control.guided_sender import GUIDED, GuidedSender  # noqa: E402
from src.logic.control.pad_locator import PadLocator, body_to_ne  # noqa: E402


class FakeVehicle:
    def __init__(self):
        self.pk = {}

    def put(self, name, data, age=0.0):
        self.pk[name] = (data, time.time() - age)

    def get_packet_with_time(self, name):
        return self.pk.get(name, (None, None))


class FakeMav:
    def __init__(self):
        self.state = FakeVehicle()
        self.pos, self.vel = [], []

    def send_position_local(self, n, e, d, yaw):
        self.pos.append((n, e, d, yaw))

    def send_velocity_body(self, f, r, d):
        self.vel.append((f, r, d))

    def disarm(self):
        pass


class FakeVision:
    def __init__(self):
        self.ls, self.pt = {}, {}

    def get_landing_shadow(self):
        return dict(self.ls)

    def get_pixel_target(self):
        return dict(self.pt)


def setup(n=0.0, e=0.0, d=-1.0, yaw=0.0, h=1.0):
    mav, vis = FakeMav(), FakeVision()
    mav.state.put("HEARTBEAT", {"custom_mode": GUIDED, "base_mode": 128})
    mav.state.put("LOCAL_POSITION_NED", {"x": n, "y": e, "z": d, "vx": 0.0, "vy": 0.0, "vz": 0.0})
    mav.state.put("ATTITUDE", {"yaw": yaw})
    vis.ls = {"h_m": h, "t": time.monotonic(), "v_fwd": 0, "v_right": 0, "v_down": 0}
    s = GuidedSender(mav, vis, log=lambda *a, **k: None, start_thread=False)
    return mav, vis, s


def see_pad(vis, fwd, right, stamp):
    vis.pt = {"detected": True, "follow": True, "pad_fwd_m": fwd, "pad_right_m": right,
              "stamp": stamp, "age_s": 0.0, "frame_age_s": 0.0}


def test_body_to_ne():
    n, e = body_to_ne(1.0, 0.0, math.pi / 2)        # facing east: forward = east
    assert abs(n) < 1e-9 and abs(e - 1.0) < 1e-9
    n, e = body_to_ne(0.0, 1.0, 0.0)                # facing north: right = east
    assert abs(n) < 1e-9 and abs(e - 1.0) < 1e-9


def test_locator_still_pad_is_still():
    loc = PadLocator(tau_s=0.0)
    # drone at (0,0) sees the pad 0.1 m ahead; then at (0.1,0) it sees it 0 m ahead: same point
    a = loc.update(0.1, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0), 0.0, 0.0)
    b = loc.update(0.0, 0.0, 0.0, (0.1, 0.0), (0.0, 0.0), 0.0, 0.1)
    assert abs(a[0] - 0.1) < 1e-9 and abs(b[0] - 0.1) < 1e-9


def test_locator_uses_position_at_frame_time():
    loc = PadLocator(tau_s=0.0)
    # packet says drone at n=0.10 moving 0.5 m/s north, frame was 0.1 s older -> drone was at 0.05
    p = loc.update(0.05, 0.0, 0.0, (0.10, 0.0), (0.5, 0.0), 0.1, 0.0)
    assert abs(p[0] - 0.10) < 1e-9


def test_locator_ignores_single_jump():
    loc = PadLocator(tau_s=0.0, jump_m=0.25, jump_n=3)
    loc.update(0.0, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0), 0.0, 0.0)
    p = loc.update(1.0, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0), 0.0, 0.1)
    assert p == (0.0, 0.0)
    loc.update(1.0, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0), 0.0, 0.2)
    p = loc.update(1.0, 0.0, 0.0, (0.0, 0.0), (0.0, 0.0), 0.0, 0.3)
    assert p == (1.0, 0.0)


def test_pos_holds_here_without_pad_and_keeps_heading():
    mav, vis, s = setup(n=1.0, e=2.0, d=-1.0, yaw=0.7, h=0.6)
    ok, _ = s.select("pos", 0.6)
    assert ok and s.stage == "pos"
    s.tick()
    n, e, d, yaw = mav.pos[-1]
    assert (round(n, 6), round(e, 6), round(d, 6)) == (1.0, 2.0, -1.0) and yaw == 0.7
    assert not mav.vel


def test_pos_goes_over_the_pad():
    mav, vis, s = setup(yaw=math.pi / 2)        # facing east
    s.select("pos", 1.0)
    see_pad(vis, 0.10, 0.0, stamp=1.0)          # pad 10 cm ahead = 10 cm east
    s.tick()
    n, e, _, _ = mav.pos[-1]
    assert abs(n) < 1e-6 and abs(e - 0.10) < 1e-6


def test_pos_target_never_far():
    mav, vis, s = setup()
    s.select("pos", 1.0)
    see_pad(vis, 2.0, 0.0, stamp=1.0)
    s.tick()
    n, e, _, _ = mav.pos[-1]
    assert abs(math.hypot(n, e) - s.p.pos_max_offset_m) < 1e-6


def test_pos_height_follows_rising_platform_rate_limited():
    mav, vis, s = setup(d=-1.0, h=0.60)
    s.select("pos", 0.60)
    t = time.monotonic()
    s.tick(now=t)
    d0 = mav.pos[-1][2]
    vis.ls["h_m"], vis.ls["t"] = 0.40, time.monotonic()   # platform rose 20 cm: too close
    s.tick(now=t + 0.1)
    d1 = mav.pos[-1][2]
    assert d1 < d0                                          # target moves UP (more negative down)
    assert abs((d0 - d1) - s.p.pos_vz_max * 0.1) < 1e-6     # at most pos_vz_max


def test_pos_brakes_without_ekf_position():
    mav, vis, s = setup()
    s.select("pos", 0.6)
    mav.state.put("LOCAL_POSITION_NED", {"x": 0, "y": 0, "z": -1, "vx": 0, "vy": 0, "vz": 0}, age=2.0)
    s.tick()
    assert mav.vel[-1] == (0.0, 0.0, 0.0) and not mav.pos


def test_pos_height_change_keeps_target():
    mav, vis, s = setup()
    s.select("pos", 0.6)
    see_pad(vis, 0.1, 0.0, stamp=1.0)
    s.tick()
    ok, _ = s.select("pos", 0.5)
    assert ok and s.pos_h == 0.5 and s._pos is not None and s.pad_locator.est is not None


def test_pos_refused_out_of_range():
    mav, vis, s = setup()
    ok, why = s.select("pos", 2.0)
    assert not ok and "height" in why


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
