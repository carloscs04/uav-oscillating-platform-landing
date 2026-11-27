"""Optical flow must not read a height change as sideways motion over the landing pad.
Run: PYTHONPATH=. python3 tests/vision/test_flow_divergence.py"""
import cv2
import numpy as np

from src.logic.vision.optical_flow import OpticalFlowEstimator


def _scene(w=640, h=360, seed=3):
    """Mostly white (pad) with texture only on the left third, like a marker off to one side."""
    rng = np.random.default_rng(seed)
    img = np.full((h, w), 235, np.uint8)
    block = (rng.random((h // 4, w // 12)) * 255).astype(np.uint8)
    block = cv2.resize(block, (w // 3, h), interpolation=cv2.INTER_NEAREST)
    img[:, :w // 3] = block
    return cv2.GaussianBlur(img, (3, 3), 0)


def _zoom(img, s, cx, cy, tx=0.0, ty=0.0):
    M = np.array([[s, 0, (1 - s) * cx + tx], [0, s, (1 - s) * cy + ty]], np.float64)
    return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]), borderValue=235)


def _rate(remove, frames):
    est = OpticalFlowEstimator(native_resolution=(640, 360), focal_px=(654.0, 654.0), work_width=320,
                               principal_px=(326.0, 160.0), remove_divergence=remove, smoothing=1.0)
    out = None
    for i, f in enumerate(frames):
        r = est.update(f, i / 30.0)
        if r is not None:
            out = r
    return out["rad_x"] * 30.0, out["rad_y"] * 30.0


def run():
    base = _scene()
    # descending 0.2 m/s at 0.3 m: the image grows ~2.2 % per frame about the optical centre
    frames = [_zoom(base, 1.0 + 0.022 * k, 326.0, 160.0) for k in range(3)]
    mx, my = _rate(False, frames)
    fx, fy = _rate(True, frames)
    print(f"zoom only: mean {mx:+.3f},{my:+.3f} rad/s  fit {fx:+.3f},{fy:+.3f} rad/s")
    assert abs(mx) > 0.15                       # the old mean reads sideways motion
    assert abs(fx) < 0.03 and abs(fy) < 0.03, (fx, fy)
    # a real translation is still measured (same with and without zoom)
    frames = [_zoom(base, 1.0, 326.0, 160.0, tx=2.0 * k) for k in range(3)]
    t_only = _rate(True, frames)
    frames = [_zoom(base, 1.0 + 0.022 * k, 326.0, 160.0, tx=2.0 * k) for k in range(3)]
    t_zoom = _rate(True, frames)
    print(f"translation: {t_only[0]:+.3f} rad/s, with zoom {t_zoom[0]:+.3f} rad/s")
    assert abs(t_only[0]) > 0.05 and abs(t_zoom[0] - t_only[0]) < 0.02
    # solver: clustered points -> fall back to the mean
    pts = np.array([[10.0, 10.0]] * 8) + np.random.default_rng(1).normal(0, 1, (8, 2))
    assert OpticalFlowEstimator.translation_without_zoom(pts, np.ones((8, 2)), 0, 0) is None
    print("PASS flow divergence")


if __name__ == "__main__":
    run()
