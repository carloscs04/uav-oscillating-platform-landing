"""
test_tracker.py - TargetTracker behaviour on synthetic frames (no hardware).

    python3 tests/vision/test_tracker.py
    python3 -m pytest tests/vision/test_tracker.py

Checks: target selection by priority, smoothing, restart after a big jump,
hold of a lost marker (with growing age) and dropping it after hold_s.
"""

import numpy as np

import _paths  # noqa: F401
from calibration import load_calibration
from src.logic.vision import ArucoDetector, CameraModel
from src.logic.vision.tracker import TargetTracker
from synthetic import MarkerRenderer

KEY = "usb_0bdc_8088_1280x720"
SIZE = 0.10


def _setup(**kw):
    model = CameraModel.from_calibration(load_calibration(KEY), "usb_0bdc_8088")
    renderer = MarkerRenderer(model.camera_matrix, model.dist_coeffs, model.image_size)
    tracker = TargetTracker(ArucoDetector("DICT_4X4_50", [10, 25, 32]), model,
                            {10: SIZE, 25: SIZE, 32: SIZE}, primary_ids=[10, 25, 32], **kw)
    return renderer, tracker


def _frame(renderer, marker_id, d, offset=(0, 0), seed=0):
    img, truth, _ = renderer.render(marker_id, SIZE, d, offset=offset, seed=seed)
    return img[..., 0], np.linalg.norm(truth)


def test_detects_and_reports_target():
    r, t = _setup(smoothing=1.0)
    gray, truth = _frame(r, 32, 1.2)
    t.update(gray, stamp=100.0)
    tg = t.target(now=100.05)
    assert tg["detected"] and tg["id"] == 32 and tg["in_view"]
    assert abs(tg["distance"] - truth) < 0.01
    assert abs(tg["age_ms"] - 50.0) < 1e-6
    assert {"x", "y", "z", "distance_raw", "reproj_err_px", "calibrated"} <= tg.keys()


def test_priority_order():
    r, t = _setup()
    a, _ = _frame(r, 32, 1.0, offset=(-0.5, 0))
    b, _ = _frame(r, 10, 1.0, offset=(0.5, 0))
    both = np.minimum(a, b)          # both markers in one image
    t.update(both, 1.0)
    tg = t.target(1.0)
    assert tg["id"] == 10 and tg["visible_ids"] == [10, 32]


def test_smoothing_and_jump_restart():
    r, t = _setup(smoothing=0.5, max_jump_m=0.3)
    g1, d1 = _frame(r, 10, 1.00)
    g2, d2 = _frame(r, 10, 1.10, seed=1)
    t.update(g1, 1.0)
    t.update(g2, 2.0)
    smoothed = t.target(2.0)["distance"]
    assert d1 + 0.02 < smoothed < d2 - 0.02          # between the two: averaged
    g3, d3 = _frame(r, 10, 1.80, seed=2)               # 0.7 m jump: restart, no averaging
    t.update(g3, 3.0)
    assert abs(t.target(3.0)["distance"] - d3) < 0.01


def test_hold_then_drop():
    r, t = _setup(hold_s=0.5)
    g, _ = _frame(r, 25, 1.0)
    empty = np.full_like(g, 110)
    t.update(g, 10.0)
    t.update(empty, 10.3)                              # lost, within hold
    tg = t.target(10.3)
    assert tg["detected"] and not tg["in_view"] and tg["age_ms"] > 250
    t.update(empty, 10.7)                              # past hold_s
    assert not t.target(10.7)["detected"]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("PASS", name)
