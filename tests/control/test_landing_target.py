"""LANDING_TARGET for ArduPilot precision landing: body-frame touchdown point and the sender."""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
os.environ.setdefault("MAVLINK20", "1")

import numpy as np  # noqa: E402

from src.logic.vision.pixel_target import body_target, tilt_compensate  # noqa: E402
from src.interfaces.MAVLink.tx.sensor_feed import MAVLinkSensorFeed  # noqa: E402


def test_body_target_undoes_tilt_compensation():
    nx, ny, r = 0.05, -0.03, 0.6
    roll, pitch = np.radians(4), np.radians(-3)
    f, rr, h = tilt_compensate(nx, ny, roll, pitch, r)
    b = body_target(None, None, None, (f, rr, h), roll, pitch)
    raw = body_target(nx, ny, r)
    assert np.allclose(np.array(b) / b[2], np.array(raw) / raw[2], atol=1e-6)   # same direction


def test_body_target_centre_is_straight_down():
    assert body_target(0.0, 0.0, 0.5) == (0.0, 0.0, 0.5)
    assert body_target(0.0, 0.0, None) is None


class _Mav:
    def __init__(self):
        self.sent = []

    def landing_target_send(self, *a):
        self.sent.append(a)


class _Master:
    def __init__(self):
        self.mav = _Mav()


class _VS:
    def __init__(self, pt):
        self.pt = pt

    def get_pixel_target(self):
        return dict(self.pt)


def test_sender_once_per_frame_and_fresh_only():
    feed = MAVLinkSensorFeed()
    m = _Master()
    st = time.monotonic()
    feed.vision_state = _VS({"detected": True, "stamp": st, "lt_x": 0.03, "lt_y": -0.02, "lt_z": 0.5})
    feed._send_landing_target(m)
    feed._send_landing_target(m)                       # same frame: not again
    assert len(m.mav.sent) == 1
    a = m.mav.sent[0]
    assert a[13] == 1 and abs(a[8] - 0.03) < 1e-9 and abs(a[9] + 0.02) < 1e-9   # position_valid, x, y
    ax, ay = a[3], a[4]                                # ArduPilot fallback: los = (-tan(ay), tan(ax), 1)
    assert abs(-np.tan(ay) - 0.03 / 0.5) < 1e-6 and abs(np.tan(ax) + 0.02 / 0.5) < 1e-6
    feed.vision_state = _VS({"detected": True, "stamp": st - 1.0, "lt_x": 0.0, "lt_y": 0.0, "lt_z": 0.5})
    feed._send_landing_target(m)                       # stale: not sent
    feed.vision_state = _VS({"detected": True, "stamp": time.monotonic(), "lt_x": None, "lt_y": None, "lt_z": None})
    feed._send_landing_target(m)                       # not allowed / no target: not sent
    assert len(m.mav.sent) == 1


if __name__ == "__main__":
    failed = 0
    for name, fn in [(k, v) for k, v in dict(globals()).items() if k.startswith("test_")]:
        try:
            fn()
            print("PASS", name)
        except Exception as e:  # noqa: BLE001
            failed += 1
            print("FAIL", name, repr(e))
    print("\nPASS" if not failed else f"\nFAIL ({failed})")
    sys.exit(1 if failed else 0)
