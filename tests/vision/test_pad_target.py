"""
test_pad_target.py - every pad marker must point at the same touchdown point (no hardware).

The pad (config landing_pad: ID 20 at 0,0 = touchdown, ID 10 at -100,0, corners 31-34) is
projected exactly through a camera mounted like the real one (forward = -x_cam,
right = -y_cam) on a drone with roll/pitch, at different heights, offsets and pad yaws.
From each marker's corners, pad_touchdown() must give the true touchdown point.

    python3 tests/vision/test_pad_target.py
    python3 -m pytest tests/vision/test_pad_target.py
"""

import sys

import numpy as np

import _paths  # noqa: F401
from src.logic.vision.pixel_target import CAM_TO_BODY, pad_touchdown

K = np.array([[500.0, 0, 326.1], [0, 500.0, 160.2], [0, 0, 1]])
DIST = np.zeros(5)
PAD = {20: ((0, 0), 25), 10: ((-100, 0), 100), 31: ((-280, 280), 50), 34: ((280, -280), 50)}


def rot(roll, pitch):
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    return (np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def project_pad(origin_fr, pad_yaw, h, roll, pitch):
    """Level frame (fwd, right, down). Pad front = pad_yaw clockwise from the drone heading."""
    front = np.array([np.cos(pad_yaw), np.sin(pad_yaw)])
    right = np.array([-np.sin(pad_yaw), np.cos(pad_yaw)])
    M = np.linalg.inv(np.array(CAM_TO_BODY))           # body (f, r) -> cam (x, y)
    Rt = rot(roll, pitch).T
    out = {}
    for mid, ((cx, cy), s) in PAD.items():
        pts = []
        for dx, dy in ((-1, 1), (1, 1), (1, -1), (-1, -1)):   # TL TR BR BL, marker y = pad front
            px, py = (cx + dx * s / 2) / 1000, (cy + dy * s / 2) / 1000
            fr = np.asarray(origin_fr) + px * right + py * front
            b = Rt @ np.array([fr[0], fr[1], h])
            xc, yc = M @ (b[:2] / b[2])
            pts.append([K[0, 0] * xc + K[0, 2], K[1, 1] * yc + K[1, 2]])
        out[mid] = np.array(pts)
    return out, h / (np.cos(roll) * np.cos(pitch))


def test_all_markers_agree():
    worst = 0.0
    for h in (0.3, 0.6, 1.0):
        for origin in ((0, 0), (0.12, -0.05), (-0.2, 0.15)):
            for yaw_d in (0, 30, 90, 180, -120):
                for roll_d, pitch_d in ((0, 0), (6, -4), (-8, 10)):
                    roll, pitch = np.radians(roll_d), np.radians(pitch_d)
                    corners, rng = project_pad(origin, np.radians(yaw_d), h, roll, pitch)
                    for mid, c in corners.items():
                        t = pad_touchdown(c, PAD[mid][0], K, DIST, roll, pitch, rng)
                        worst = max(worst, abs(t[0] - origin[0]), abs(t[1] - origin[1]))
    assert worst < 1e-6, worst


def test_id10_alone_is_offset_10cm():
    # Drone straight above ID 20: ID 10's own centre is 10 cm to the LEFT (pad unrotated),
    # its touchdown estimate is 0, 0. (Seen in the 2026-11-22 log: right = -11 cm for ID 10.)
    corners, rng = project_pad((0, 0), 0.0, 1.0, 0.0, 0.0)
    t = pad_touchdown(corners[10], (-100, 0), K, DIST, 0.0, 0.0, rng)
    assert abs(t[0]) < 1e-6 and abs(t[1]) < 1e-6


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
