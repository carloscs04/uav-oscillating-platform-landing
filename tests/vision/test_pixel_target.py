"""
test_pixel_target.py - check the control measurement on synthetic images (no hardware).

The landing controller uses: x, y from the PIXEL position of the marker centre relative to
the optical axis (any marker, size not used), z from the lidar. This renders markers
through the onboard camera's REAL calibration at known positions and checks that

    x_m = nx * h,  y_m = ny * h     (h = true depth, standing in for the lidar)

matches the truth, for the pad family (any ID) and an AprilTag, flat and tilted.

    python3 tests/vision/test_pixel_target.py            # table + PASS/FAIL
    python3 -m pytest tests/vision/test_pixel_target.py  # same checks under pytest

Tolerance: max(5 mm, 1.5 % of the height). Views where the marker does not fit are skipped.
"""

import sys

import cv2
import numpy as np

import _paths  # noqa: F401
from src.inputs.camera_driver.calib_params import load_calibration
from src.logic.vision import CameraModel, get_dictionary
from src.logic.vision.pixel_target import marker_center_px, pixel_target
from synthetic import MarkerRenderer

CALIB = "usb_0bdc_8088_640x360"
RESOLUTION = (640, 360)
HEIGHTS = [0.15, 0.3, 0.5, 1.0, 1.5]
OFFSETS = [(0, 0), (0.5, 0.0), (0.0, 0.5), (-0.6, -0.4), (0.7, 0.6)]
TILTS = [(0, 0), (10, 0), (0, 15)]          # drone tilt seen as a tilted marker
CASES = [  # (dictionary, marker id, size m)
    ("DICT_4X4_50", 10, 0.100),
    ("DICT_4X4_50", 20, 0.0249),
    ("DICT_4X4_50", 33, 0.050),             # not a pad ID: must work too
    ("DICT_APRILTAG_36h11", 3, 0.100),      # other family
]


def tolerance(h):
    return max(0.005, 0.015 * h)


def _detect(img, dictionary):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    det = cv2.aruco.ArucoDetector(get_dictionary(dictionary), cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(gray)
    return [] if ids is None else [(int(i), c.reshape(4, 2)) for i, c in zip(ids.ravel(), corners)]


def run():
    model = CameraModel.from_calibration(load_calibration(CALIB), CALIB)
    K, dist = model.camera_matrix, model.dist_coeffs
    renderers = {}
    rows, failures = [], []
    for dictionary, marker_id, size in CASES:
        r = renderers.setdefault(dictionary, MarkerRenderer(K, dist, RESOLUTION, dictionary))
        for h in HEIGHTS:
            errs, skipped = [], 0
            for i, tilt in enumerate(TILTS):
                for j, off in enumerate(OFFSETS):
                    img, tvec, true_corners = r.render(marker_id, size, h, tilt_deg=tilt, offset=off,
                                                       roll_deg=23 * (i + j), seed=7 * i + j)
                    if not r.fully_visible(true_corners) or np.min(
                            np.linalg.norm(true_corners - np.roll(true_corners, 1, 0), axis=1)) < 14:
                        skipped += 1
                        continue
                    where = f"{dictionary} #{marker_id} h={h} tilt={tilt} off={off}"
                    found = [c for i_, c in _detect(img, dictionary) if i_ == marker_id]
                    if not found:
                        failures.append(f"{where}: not detected")
                        continue
                    depth = float(tvec[2])          # lidar stand-in: distance along the axis
                    t = pixel_target(found[0], K, dist, depth)
                    e = float(np.hypot(t["x_m"] - tvec[0], t["y_m"] - tvec[1]))
                    errs.append(e)
                    if e > tolerance(depth):
                        failures.append(f"{where}: error {e * 1000:.1f} mm (limit {tolerance(depth) * 1000:.0f})")
            e = np.array(errs) if errs else np.array([np.nan])
            rows.append((f"{dictionary.replace('DICT_', '')} #{marker_id}", h, len(errs), skipped,
                         np.nanmean(e) * 1000, np.nanmax(e) * 1000, tolerance(h) * 1000))
    return rows, failures


def check_center_under_perspective():
    """Diagonal intersection is the projected centre; the corner mean is not."""
    model = CameraModel.from_calibration(load_calibration(CALIB), CALIB)
    r = MarkerRenderer(model.camera_matrix, model.dist_coeffs, RESOLUTION)
    _, tvec, corners = r.render(10, 0.1, 0.25, tilt_deg=(35, 0))
    true_c, _ = cv2.projectPoints(np.zeros((1, 3)), np.zeros(3), tvec, model.camera_matrix, model.dist_coeffs)
    true_c = true_c.reshape(2)
    diag = np.linalg.norm(marker_center_px(corners) - true_c)
    mean = np.linalg.norm(corners.mean(axis=0) - true_c)
    return diag, mean


def test_pixel_target():
    rows, failures = run()
    assert not failures, "\n".join(failures)


def test_center_diagonals():
    diag, mean = check_center_under_perspective()
    assert diag < 0.5, f"diagonal centre off by {diag:.2f} px"


if __name__ == "__main__":
    rows, failures = run()
    print(f"{'marker':<22}{'h m':>6}{'n':>4}{'skip':>5}{'mean mm':>9}{'max mm':>8}{'limit':>7}")
    for name, h, n, sk, mean, mx, lim in rows:
        print(f"{name:<22}{h:>6.2f}{n:>4}{sk:>5}{mean:>9.2f}{mx:>8.2f}{lim:>7.1f}")
    diag, mean = check_center_under_perspective()
    print(f"\ncentre under 35 deg tilt at 0.25 m: diagonals {diag:.2f} px, corner mean {mean:.2f} px")
    ok = not failures and diag < 0.5
    for f in failures[:30]:
        print("FAIL", f)
    print("\nPASS" if ok else f"\nFAIL ({len(failures)} views)")
    sys.exit(0 if ok else 1)
