"""
test_target_select.py - which marker is the target (no hardware).

    known markers visible   -> highest in known_markers.yaml wins, unknowns ignored
    only unknown markers    -> nearest the centre cross, kept while visible (no jumping)
    unknown target          -> follow only after `followunknown yes`

    python3 tests/vision/test_target_select.py
    python3 -m pytest tests/vision/test_target_select.py
"""

import sys
import time
import types

import numpy as np

import _paths  # noqa: F401
from src.logic.vision.target_select import load_registry, normalize_label, select_target
from src.logic.vision.vision_state import VisionState

PP = (320.0, 180.0)


def square(cx, cy, s=40):
    h = s / 2
    return np.array([[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]], np.float32)


def test_normalize():
    assert normalize_label("DICT_APRILTAG_36h11 #3") == "APRILTAG_36h11 #3"
    assert normalize_label("apriltag_36h11#3") == "APRILTAG_36h11 #3"
    assert normalize_label(" 4x4_50 # 10 ") == "4X4_50 #10"


def test_registry_file():
    reg = load_registry()
    assert reg and all("#" in r for r in reg), reg


def test_known_by_priority():
    reg = ["4X4_50 #20", "APRILTAG_36h11 #3"]
    cands = [("APRILTAG_36h11 #3", square(330, 185)), ("5X5_1000 #7", square(321, 181)),
             ("4X4_50 #20", square(500, 300))]
    assert select_target(cands, reg, PP) == (2, True)          # #20 first in the list, though far
    assert select_target(cands[:2], reg, PP) == (0, True)      # known beats a nearer unknown


def test_unknown_nearest_and_sticky():
    cands = [("5X5_1000 #7", square(500, 300)), ("APRILTAG_36h11 #9", square(340, 190))]
    assert select_target(cands, [], PP) == (1, False)          # nearest the cross
    moved = [("5X5_1000 #7", square(325, 182)), ("APRILTAG_36h11 #9", square(360, 200))]
    assert select_target(moved, [], PP, "APRILTAG_36h11 #9") == (1, False)  # keeps it
    assert select_target(moved[:1], [], PP, "APRILTAG_36h11 #9") == (0, False)  # gone -> other
    assert select_target([], [], PP) == (None, False)


def _node():
    """CameraNode without a camera: fake model/tracker, real selection + publishing code."""
    from src.inputs.camera_driver.camera_node import CameraNode
    vs = VisionState()
    height = types.SimpleNamespace(sensor_status="OK", distance_m=0.8)
    node = CameraNode(types.SimpleNamespace(), vs, height_state=height)
    K = np.array([[500, 0, PP[0]], [0, 500, PP[1]], [0, 0, 1]], np.float64)
    node.model = types.SimpleNamespace(camera_matrix=K, dist_coeffs=np.zeros(5))
    node.tracker = types.SimpleNamespace(detector=types.SimpleNamespace(last_all=[]), sizes_m={})
    node.registry = ["4X4_50 #20", "4X4_50 #10"]
    vs.scan_any = True
    return node, vs


def test_node_flow():
    node, vs = _node()
    now = time.monotonic()
    shape = (360, 640)

    # Only an unknown AprilTag: yellow candidate, asks, not followed
    node._scan_found = [("APRILTAG_36h11 #3", square(370, 180), now)]
    node._publish_overlay([], shape)
    pt = vs.get_pixel_target()
    assert pt["source"] == "APRILTAG_36h11 #3" and not pt["known"]
    assert pt["approval"] == "ask" and pt["follow"] is False
    assert abs(pt["x_m"] - 50 / 500 * 0.8) < 1e-3                # 50 px at f=500, h=0.8 m -> 8 cm
    assert vs.overlay["target"] == "APRILTAG_36h11 #3" and vs.overlay["items"][0]["k"] is False

    vs.follow_unknown = True
    node._publish_overlay([], shape)
    assert vs.get_pixel_target()["follow"] is True
    vs.follow_unknown = False
    node._publish_overlay([], shape)
    assert vs.get_pixel_target()["approval"] == "no" and vs.get_pixel_target()["follow"] is False

    # Known 4X4 #10 and #20 appear (same family, any ID from the main detector): #20 wins
    node.tracker.detector.last_all = [(10, square(200, 100)), (20, square(450, 250))]
    node._publish_overlay([], shape)
    pt = vs.get_pixel_target()
    assert pt["source"] == "4X4_50 #20" and pt["known"] and pt["follow"]
    items = {i["label"]: i["k"] for i in vs.overlay["items"]}
    assert items == {"4X4_50 #10": True, "4X4_50 #20": True, "APRILTAG_36h11 #3": False}

    # Nothing in view
    node.tracker.detector.last_all, node._scan_found = [], []
    node._publish_overlay([], shape)
    assert vs.get_pixel_target() == {"detected": False}


def test_phantom_and_stale_never_target():
    """Seen 2026-11-22: the scan misread the 4x4 #20 as ARUCO_ORIGINAL #193; when #20 dropped
    for one frame the held (1 s old) phantom became the target."""
    node, vs = _node()
    shape, now = (360, 640), time.monotonic()
    node.tracker.detector.last_all = [(20, square(300, 200))]
    node._scan_found = [("ARUCO_ORIGINAL #193", square(303, 198), now)]
    node._publish_overlay([], shape)
    assert vs.get_pixel_target()["source"] == "4X4_50 #20"
    assert [i["label"] for i in vs.overlay["items"]] == ["4X4_50 #20"]       # phantom dropped
    # #20 missing for a frame, phantom still held by the scan -> no target, not the phantom
    node.tracker.detector.last_all = []
    node._publish_overlay([], shape)
    assert vs.get_pixel_target() == {"detected": False}
    # a real unknown elsewhere, but #20 was lost < 0.5 s ago -> still no switch
    node._scan_found = [("APRILTAG_36h11 #3", square(500, 100), time.monotonic())]
    node._publish_overlay([], shape)
    assert vs.get_pixel_target() == {"detected": False}
    # known gone for > 0.5 s -> the unknown may become the target
    node._known_seen = {k: (c, s, t - 1.0) for k, (c, s, t) in node._known_seen.items()}
    node._scan_found = [("APRILTAG_36h11 #3", square(500, 100), time.monotonic())]
    node._publish_overlay([], shape)
    assert vs.get_pixel_target()["source"] == "APRILTAG_36h11 #3"
    # a held (stale) scan result is drawn but never the target
    node._scan_found = [("APRILTAG_36h11 #3", square(500, 100), time.monotonic() - 0.6)]
    node._publish_overlay([], shape)
    assert vs.get_pixel_target() == {"detected": False}
    assert vs.overlay["items"][0]["label"] == "APRILTAG_36h11 #3"


def test_node_shadow_law():
    """Pad marker in view, level drone: the law (shadow) must move TOWARDS the touchdown point,
    and ID 10 seen alone must give the same target as ID 20 (pad offset applied)."""
    node, vs = _node()
    node.registry = ["4X4_50 #10", "4X4_50 #20"]
    node.pad_centers = {20: (0.0, 0.0), 10: (-100.0, 0.0)}
    node.vehicle_state = types.SimpleNamespace(get_packet_with_time=lambda k: ({"roll": 0.0, "pitch": 0.0}, time.time()))
    shape = (360, 640)
    # ID 20 to the RIGHT in the image (u > cx): x_cam > 0 -> marker BEHIND the drone
    node.tracker.detector.last_all = [(20, square(370, 180))]
    node._publish_overlay([], shape)
    pt, ls = vs.get_pixel_target(), vs.get_landing_shadow()
    assert pt["pad_fwd_m"] is not None and abs(pt["pad_fwd_m"] - pt["fwd_m"]) < 1e-3   # ID 20 = touchdown
    assert ls["phase"] == "ALIGN" and ls["v_fwd"] < 0 and ls["target"] == "pad"
    # Panel arrow: image x = -fwd -> v_fwd < 0 draws to the right, towards the marker. OK.
    # ID 10 alone, 100 mm to the pad's left of a touchdown point that is straight below:
    # f=500 px, h=0.8 m -> 0.1 m = 62.5 px. Pad front = drone forward = -x_img, pad right =
    # drone right = -y_img, so pad left = +y_img (down in the image).
    # Printed top edge towards the pad FRONT = drone forward = image LEFT (-x); marker right =
    # clockwise from that seen from above = image UP (-y). Corners TL, TR, BR, BL:
    cx, cy, k = PP[0], PP[1] + 62.5, 20
    pad10 = np.array([[cx - k, cy + k], [cx - k, cy - k], [cx + k, cy - k], [cx + k, cy + k]], np.float32)
    node.tracker.detector.last_all = [(10, pad10)]
    node._known_seen = {}
    node._publish_overlay([], shape)
    pt = vs.get_pixel_target()
    assert pt["source"] == "4X4_50 #10"
    assert abs(pt["right_m"] + 0.10) < 2e-3, pt["right_m"]            # marker centre 10 cm left
    assert abs(pt["pad_fwd_m"]) < 0.01 and abs(pt["pad_right_m"]) < 0.01, (pt["pad_fwd_m"], pt["pad_right_m"])


if __name__ == "__main__":
    tests = [v for k, v in dict(globals()).items() if k.startswith("test_")]
    failed = 0
    for t in tests:
        try:
            t()
            print("PASS", t.__name__)
        except Exception as e:  # noqa: BLE001
            failed += 1
            print("FAIL", t.__name__, repr(e))
    print("\nPASS" if not failed else f"\nFAIL ({failed})")
    sys.exit(1 if failed else 0)
