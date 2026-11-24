"""
test_height_check.py - camera-only height (marker plane) and the camlidar summary (no hardware).

    python3 tests/vision/test_height_check.py
    python3 -m pytest tests/vision/test_height_check.py
"""

import random

import cv2
import numpy as np

import _paths  # noqa: F401
from src.logic.vision.height_check import plane_height, summarize


def _pose(h, roll_deg, pitch_deg, off=(0.05, -0.03)):
    """Camera tilted by roll/pitch above a flat marker at height h: marker pose in the camera."""
    r, p = np.radians(roll_deg), np.radians(pitch_deg)
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    R_cw = rx @ ry                      # camera axes in the world (world z = down to the floor)
    t_world = np.array([off[0], off[1], h])
    R_mc = R_cw.T                       # marker (axes = world) seen from the camera
    rvec, _ = cv2.Rodrigues(R_mc)
    return rvec.reshape(3), R_mc @ t_world


def test_plane_height_ignores_tilt_and_offset():
    for h in (0.12, 0.35, 1.2):
        for roll, pitch in ((0, 0), (8, -5), (-12, 10)):
            rvec, tvec = _pose(h, roll, pitch)
            assert abs(plane_height(rvec, tvec) - h) < 1e-6, (h, roll, pitch)


def test_summary_ok_and_biased():
    rnd = random.Random(1)
    s = []
    for _ in range(600):
        hl = rnd.uniform(0.12, 0.55)
        s.append((20, hl + rnd.gauss(0, 0.004), hl))                  # good marker
        hl2 = rnd.uniform(0.35, 1.4)
        s.append((10, 1.05 * hl2 + 0.01 + rnd.gauss(0, 0.004), hl2))  # 5 % scale error
    lines, v = summarize(s)
    assert v[20]["ok"], "\n".join(lines)
    assert not v[10]["ok"], "\n".join(lines)
    assert abs(v[10]["k"] - 1.05) < 0.01 and abs(v[10]["c_m"] - 0.01) < 0.005


def test_summary_empty():
    lines, v = summarize([(10, None, 0.5), (20, 0.3, None)])
    assert v == {} and "no samples" in lines[0]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("PASS", name)
    _lines, _ = summarize([(20, 0.301, 0.30)] * 20 + [(10, 0.82, 0.80)] * 20)
    print("\n".join(_lines))
