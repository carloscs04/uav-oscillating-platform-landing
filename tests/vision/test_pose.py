"""
test_pose.py - check marker distance accuracy on synthetic images (no hardware).

Renders the flight marker through each camera's REAL calibration at known
distances, tilts and positions in the image, runs the detector + pose code,
and compares the result with the truth.

    python3 tests/vision/test_pose.py            # table + PASS/FAIL
    python3 -m pytest tests/vision/test_pose.py  # same checks under pytest

Tolerance per view:
  * up to the 2 m working height: max(5 mm, 1 % of the distance)
  * beyond it (margin check):     2 % of the distance
Views where the marker does not fit in the image are skipped (not failures).
Synthetic images are ideal apart from blur and noise; real-world error will be
larger (calibration, print size, lighting): that is what Stage 5 measures.
"""

import sys

import numpy as np

import _paths  # noqa: F401
from calibration import load_calibration
from cameras import get_camera, load_config
from src.logic.vision import ArucoDetector, CameraModel, estimate_pose
from synthetic import MarkerRenderer

WORKING_MAX_M = 2.0
DISTANCES = [0.3, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
TILTS = [(0, 0), (30, 0), (0, 45), (35, 35)]
OFFSETS = [(0, 0), (0.6, 0.5), (-0.6, -0.5)]


def tolerance(d):
    return max(0.005, 0.01 * d) if d <= WORKING_MAX_M else 0.02 * d


def run_camera(name, config, refine="edges"):
    cam = get_camera(name, config)
    model = CameraModel.from_calibration(load_calibration(cam.calib_key), name)
    acfg = config["aruco"]
    size_m = acfg["marker_mm"] / 1000.0
    marker_id = acfg["ids"][0] if acfg.get("ids") else 10
    det = ArucoDetector(acfg["dictionary"], acfg.get("ids"), refine=refine)
    renderer = MarkerRenderer(model.camera_matrix, model.dist_coeffs, cam.resolution,
                              acfg["dictionary"])

    rows, failures = [], []
    for d in DISTANCES:
        errs, skipped = [], 0
        for i, tilt in enumerate(TILTS):
            for j, off in enumerate(OFFSETS):
                img, truth, true_corners = renderer.render(
                    marker_id, size_m, d, tilt_deg=tilt, offset=off,
                    roll_deg=17 * (i + j), seed=i * 10 + j)
                if not renderer.fully_visible(true_corners):
                    skipped += 1
                    continue
                where = f"{name} d={d} tilt={tilt} off={off}"
                found = det.detect(img)
                if not found:
                    failures.append(f"{where}: not detected")
                    continue
                m = estimate_pose(*found[0], size_m, model)
                err = m.distance - np.linalg.norm(truth)
                errs.append(err)
                if abs(err) > tolerance(d):
                    failures.append(f"{where}: error {err * 1000:+.1f} mm "
                                    f"(limit {tolerance(d) * 1000:.0f})")
                if not m.reliable:
                    failures.append(f"{where}: flagged unreliable "
                                    f"(reprojection {m.reproj_err_px:.2f} px)")
        e = np.array(errs) if errs else np.array([np.nan])
        px = size_m * model.camera_matrix[0, 0] / d
        rows.append((d, px, len(errs), skipped, np.mean(e) * 1000, np.std(e) * 1000,
                     np.max(np.abs(e)) * 1000, 100 * np.max(np.abs(e)) / d,
                     tolerance(d) * 1000))
    return rows, failures


def print_table(name, rows):
    n = len(TILTS) * len(OFFSETS)
    print(f"\n{name}: distance error, {n} views per distance "
          f"(tilt up to 50 deg, centre and corners of the image)")
    print(f"{'dist':>6} {'marker':>7} {'views':>6} {'skipped':>8} "
          f"{'mean':>8} {'spread':>7} {'worst':>8} {'worst%':>7} {'limit':>7}")
    for d, px, ok, skipped, mean, std, worst, pct, lim in rows:
        mark = "" if d <= WORKING_MAX_M else "  (beyond 2 m)"
        print(f"{d:5.1f}m {px:5.0f}px {ok:6d} {skipped:8d} {mean:+6.1f}mm {std:5.1f}mm "
              f"{worst:6.1f}mm {pct:6.2f}% {lim:5.0f}mm{mark}")


def _check(name):
    rows, failures = run_camera(name, load_config())
    print_table(name, rows)
    for f in failures:
        print("FAIL", f)
    return failures


def test_imx219():
    assert not _check("csi_imx219")


def test_usb():
    assert not _check("usb_0bdc_8088")


if __name__ == "__main__":
    config = load_config()
    all_failures = []
    for name in config["cameras"]:
        try:
            rows, failures = run_camera(name, config)
        except FileNotFoundError as exc:
            print(f"\n{name}: skipped ({exc})")
            continue
        print_table(name, rows)
        all_failures += failures
    print()
    for f in all_failures:
        print("FAIL", f)
    print("PASS" if not all_failures else f"{len(all_failures)} problem(s)")
    sys.exit(1 if all_failures else 0)
