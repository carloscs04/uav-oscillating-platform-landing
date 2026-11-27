"""Pad marker fusion: height hand-over 10 -> 20 and no step when a marker appears / is lost.
Run: PYTHONPATH=. python3 tests/vision/test_pad_fusion.py"""
from src.logic.vision.pad_fusion import PadFusion

A, B = (0.00, 0.00), (0.012, -0.005)          # ID 10 and ID 20 disagree by ~1.3 cm


def run():
    f = PadFusion(near_id=20, far_id=10)
    t, dt = 0.0, 1 / 30
    out = f.update({10: A}, 1.0, t)
    assert out == A
    prev = out
    worst = 0.0
    # descend 1.0 -> 0.15 m; ID 20 appears at 0.75 m, ID 10 lost at 0.65 m
    h = 1.0
    while h > 0.15:
        t += dt
        h -= 0.3 * dt
        pts = {}
        if h > 0.65:
            pts[10] = A
        if h < 0.75:
            pts[20] = B
        out = f.update(pts, h, t)
        step = ((out[0] - prev[0]) ** 2 + (out[1] - prev[1]) ** 2) ** 0.5
        worst = max(worst, step)
        prev = out
    assert worst < 0.0015, worst                  # < 1.5 mm per frame (was 13 mm in one frame)
    for _ in range(60):                           # 2 s more on ID 20 only: offset faded
        t += dt
        out = f.update({20: B}, 0.15, t)
    assert abs(out[0] - B[0]) < 1e-3 and abs(out[1] - B[1]) < 1e-3, out   # ends on ID 20
    # weights
    assert f.weight(10, 0.8) == 1.0 and f.weight(20, 0.8) == 0.0
    assert f.weight(20, 0.6) == 1.0 and f.weight(10, 0.6) == 0.0
    # only the small marker high up: still used
    f.reset()
    assert f.update({20: B}, 1.2, 0.0) == B
    # after a long gap no old offset is carried
    f.reset()
    f.update({10: A}, 1.0, 0.0)
    assert f.update({20: B}, 0.2, 5.0) == B
    print(f"PASS pad fusion (max step {1000 * worst:.2f} mm/frame)")


if __name__ == "__main__":
    run()
