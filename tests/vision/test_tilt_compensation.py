"""
test_tilt_compensation.py - marker offset from the drone centre with the drone tilted (no hardware).

A marker at a known level-frame position (forward, right) and height is seen by a drone with
roll/pitch: build the camera ray and the lidar slant range the drone would measure, then
check tilt_compensate() gives the true position back. Also checks the size of the error it
removes (~h*tan(tilt)), and the camera->body mapping measured by hand on 2026-11-22.

    python3 tests/vision/test_tilt_compensation.py
    python3 -m pytest tests/vision/test_tilt_compensation.py
"""

import sys
import time
import types

import numpy as np

import _paths  # noqa: F401
from src.logic.vision.pixel_target import CAM_TO_BODY, tilt_compensate


def rot(roll, pitch):
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    return (np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def simulate(fwd, right, h, roll, pitch):
    """What the drone measures: camera ray (nx, ny) and lidar slant range."""
    body = rot(roll, pitch).T @ np.array([fwd, right, h])     # level -> body (fwd, right, down)
    bx, by = body[0] / body[2], body[1] / body[2]
    # invert CAM_TO_BODY (forward = -x_cam, right = -y_cam)
    M = np.array(CAM_TO_BODY)
    nx, ny = np.linalg.solve(M, [bx, by])
    return nx, ny, h / (np.cos(roll) * np.cos(pitch))


def test_level_matches_hand_test():
    # hand test: drone moved 10 cm forward -> marker 10 cm BEHIND -> x_cam = +0.1 at 1 m
    f, r, h = tilt_compensate(0.1, 0.0, 0.0, 0.0, 1.0)
    assert abs(f + 0.1) < 1e-9 and abs(r) < 1e-9 and abs(h - 1.0) < 1e-9
    f, r, _ = tilt_compensate(0.0, 0.1, 0.0, 0.0, 1.0)      # moved right -> marker on the left
    assert abs(r + 0.1) < 1e-9


def test_recovers_truth_tilted():
    worst = 0.0
    for h in (0.2, 0.5, 1.0, 2.0):
        for fwd, right in ((0, 0), (0.1, 0), (0, -0.15), (-0.2, 0.1)):
            for roll_d, pitch_d in ((0, 0), (10, 0), (0, 10), (-15, 8), (20, -20)):
                roll, pitch = np.radians(roll_d), np.radians(pitch_d)
                nx, ny, rng = simulate(fwd, right, h, roll, pitch)
                f, r, hv = tilt_compensate(nx, ny, roll, pitch, rng)
                worst = max(worst, abs(f - fwd), abs(r - right), abs(hv - h))
    assert worst < 1e-9, worst


def test_error_removed_is_h_tan_tilt():
    # marker straight below, drone pitched 10 deg nose-down at 0.8 m: uncompensated ~14 cm off
    nx, ny, rng = simulate(0.0, 0.0, 0.8, 0.0, np.radians(-10))
    raw_f, _, _ = tilt_compensate(nx, ny, 0.0, 0.0, rng)
    f, _, _ = tilt_compensate(nx, ny, 0.0, np.radians(-10), rng)
    # nose down: the camera looks backwards, the marker below appears IN FRONT
    assert abs(raw_f - 0.8 * np.tan(np.radians(10))) < 0.005, raw_f
    assert abs(f) < 1e-9
    # right wing down (roll +): the marker below appears to the RIGHT
    nx, ny, rng = simulate(0.0, 0.0, 0.8, np.radians(10), 0.0)
    _, raw_r, _ = tilt_compensate(nx, ny, 0.0, 0.0, rng)
    assert raw_r > 0.13, raw_r


def test_node_uses_attitude():
    from test_target_select import _node, square
    node, vs = _node()
    att = {"roll": np.radians(5.0), "pitch": np.radians(-3.0)}
    node.vehicle_state = types.SimpleNamespace(get_packet_with_time=lambda k: (att, time.time()))
    node._scan_found = [("APRILTAG_36h11 #3", square(370, 180), time.monotonic())]
    node._publish_overlay([], (360, 640))
    pt = vs.get_pixel_target()
    assert pt["roll_deg"] == 5.0 and pt["pitch_deg"] == -3.0 and pt["fwd_m"] is not None
    assert abs(pt["fwd_raw_m"] + pt["x_m"]) < 1e-3       # raw forward = -x_cam
    # stale attitude -> no compensated values
    node.vehicle_state = types.SimpleNamespace(get_packet_with_time=lambda k: (att, time.time() - 1.0))
    node._publish_overlay([], (360, 640))
    assert vs.get_pixel_target()["fwd_m"] is None


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
