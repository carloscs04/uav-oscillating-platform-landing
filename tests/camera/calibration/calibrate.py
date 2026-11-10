"""
calibrate.py - compute camera intrinsics + distortion from captured ChArUco images.

    python3 -m calibration.calibrate                        # active_camera from config.yaml
    python3 -m calibration.calibrate --camera usb_0bdc_8088
    python3 -m calibration.calibrate --rational             # 8-coefficient model for wide lenses

Reads   calibration/images/<calib_key>/img_*.png
Writes  calibration/params/<calib_key>.yaml          (used by the pose code)
        calibration/params/<calib_key>_check.jpg     (original | undistorted, to check by eye)

Quality guide (RMS reprojection error):  < 0.5 px good,  0.5-1.0 px usable,  > 1.0 px redo.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

if __package__ in (None, ""):  # allow `python3 calibration/calibrate.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from calibration.board import (board_config, describe, detect_corners,  # noqa: E402
                               make_board, make_detector, n_corners)
from calibration.capture import GRID  # noqa: E402
from calibration.params import (PARAMS_DIR, images_dir,  # noqa: E402
                                save_calibration)
from cameras import get_camera, load_config  # noqa: E402

MIN_IMAGES = 10
LAYOUT_KEYS = ("squares_x", "squares_y", "dictionary")


def detect_all(paths, detector, board, image_size, min_corners):
    """Return list of (path, obj_points, img_points, corners) for usable images."""
    views = []
    for path in paths:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"  {path.name}: unreadable, skipped")
            continue
        if (gray.shape[1], gray.shape[0]) != tuple(image_size):
            print(f"  {path.name}: size {gray.shape[1]}x{gray.shape[0]} != "
                  f"{image_size[0]}x{image_size[1]}, skipped")
            continue
        corners, ids = detect_corners(detector, gray)
        n = 0 if ids is None else len(ids)
        if n < min_corners:
            print(f"  {path.name}: {n} corners (< {min_corners}), skipped")
            continue
        obj, img = board.matchImagePoints(corners, ids)
        views.append((path, obj, img, corners))
    return views


def solve(views, image_size, flags):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 100, 1e-9)
    rms, K, dist, _, _, std_intr, _, per_view = cv2.calibrateCameraExtended(
        [v[1] for v in views], [v[2] for v in views], tuple(image_size),
        None, None, flags=flags, criteria=criteria)
    return rms, K, dist, std_intr.ravel(), per_view.ravel()


def coverage_percent(views, image_size):
    w, h = image_size
    hit = np.zeros((GRID[1], GRID[0]), dtype=bool)
    for v in views:
        pts = v[3].reshape(-1, 2)
        gx = np.clip((pts[:, 0] / w * GRID[0]).astype(int), 0, GRID[0] - 1)
        gy = np.clip((pts[:, 1] / h * GRID[1]).astype(int), 0, GRID[1] - 1)
        hit[gy, gx] = True
    return 100.0 * hit.mean()


def write_check_image(path, image_path, K, dist, image_size):
    """Side by side: original | undistorted (straight lines should look straight)."""
    img = cv2.imread(str(image_path))
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist, tuple(image_size), alpha=0)
    und = cv2.undistort(img, K, dist, None, new_K)
    both = np.hstack([img, und])
    for x, label in ((10, "original"), (img.shape[1] + 10, "undistorted")):
        cv2.putText(both, label, (x, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
    scale = min(1.0, 2400 / both.shape[1])
    both = cv2.resize(both, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(path), both, [cv2.IMWRITE_JPEG_QUALITY, 90])


def verdict(rms):
    if rms < 0.5:
        return "GOOD"
    if rms < 1.0:
        return "USABLE (recapture for better accuracy if possible)"
    return "POOR - recapture: hold the board still, keep it flat, check focus"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", help="camera name from config.yaml (default: active_camera)")
    ap.add_argument("--config", help="path to config.yaml")
    ap.add_argument("--rational", action="store_true",
                    help="use the 8-coefficient rational model (wide-angle lenses)")
    ap.add_argument("--no-reject", action="store_true", help="keep every view")
    args = ap.parse_args()

    config = load_config(args.config)
    bcfg = board_config(config)
    ccfg = config.get("calibration", {})
    min_corners = int(ccfg.get("min_corners", 12))
    max_view_err = float(ccfg.get("max_view_error_px", 1.0))

    cam = get_camera(args.camera, config)  # not opened: only its name and resolution
    key, image_size = cam.calib_key, cam.resolution
    folder = images_dir(key)
    paths = sorted(folder.glob("img_*.png"))
    if not paths:
        sys.exit(f"No images in {folder}. Run: python3 -m calibration.capture --camera {cam.camera_id}")

    capture_info = {}
    if (folder / "camera.yaml").exists():
        capture_info = yaml.safe_load(open(folder / "camera.yaml")) or {}
        cap_board = capture_info.get("board", {})
        changed = [k for k in LAYOUT_KEYS if k in cap_board and cap_board[k] != bcfg[k]]
        if changed:
            sys.exit(f"Board layout in config.yaml differs from the one used for capture "
                     f"({', '.join(changed)}). Recapture or restore the config.")

    print(f"Camera: {key}   Board: {describe(bcfg)} ({n_corners(bcfg)} corners)")
    print(f"Detecting board in {len(paths)} images from {folder} ...")
    board = make_board(bcfg)
    views = detect_all(paths, make_detector(board), board, image_size, min_corners)
    if len(views) < MIN_IMAGES:
        sys.exit(f"Only {len(views)} usable images (need {MIN_IMAGES}+). Capture more.")

    flags = cv2.CALIB_RATIONAL_MODEL if args.rational else 0
    rms, K, dist, std, per_view = solve(views, image_size, flags)
    print(f"First pass: {len(views)} views, RMS {rms:.3f} px")

    rejected = []
    if not args.no_reject:
        med = float(np.median(per_view))
        bad = [i for i, e in enumerate(per_view) if e > max_view_err and e > 2 * med]
        if bad and len(views) - len(bad) >= MIN_IMAGES:
            rejected = [(views[i][0].name, float(per_view[i])) for i in bad]
            for name, err in rejected:
                print(f"  rejecting {name}: {err:.3f} px (median {med:.3f})")
            views = [v for i, v in enumerate(views) if i not in set(bad)]
            rms, K, dist, std, per_view = solve(views, image_size, flags)
            print(f"Second pass: {len(views)} views, RMS {rms:.3f} px")

    w, h = image_size
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    hfov = np.degrees(2 * np.arctan(w / (2 * fx)))
    vfov = np.degrees(2 * np.arctan(h / (2 * fy)))
    cov = coverage_percent(views, image_size)

    print("\n=== Result ===")
    print(f"RMS reprojection error: {rms:.3f} px  -> {verdict(rms)}")
    print(f"fx={fx:.1f} (+/-{std[0]:.1f})  fy={fy:.1f} (+/-{std[1]:.1f})  "
          f"cx={cx:.1f} (+/-{std[2]:.1f})  cy={cy:.1f} (+/-{std[3]:.1f})")
    print(f"Distortion: {np.array2string(dist.ravel(), precision=4, suppress_small=True)}")
    print(f"Field of view: {hfov:.1f} x {vfov:.1f} deg   Image coverage: {cov:.0f}%")
    worst = np.argsort(per_view)[::-1][:5]
    print("Worst views: " + ", ".join(f"{views[i][0].name} {per_view[i]:.2f}px" for i in worst))
    if cov < 70:
        print("Warning: corners cover less than 70% of the image. Distortion near the "
              "edges may be wrong; capture more views with the board near the edges.")

    meta = {
        "camera": capture_info.get("camera", cam.info()),
        "board": dict(bcfg),
        "model": "rational (8 coeffs)" if args.rational else "standard (k1 k2 p1 p2 k3)",
        "fov_deg": [round(float(hfov), 2), round(float(vfov), 2)],
        "std_dev": {k: round(float(v), 3) for k, v in zip(("fx", "fy", "cx", "cy"), std[:4])},
        "coverage_percent": round(float(cov), 1),
        "n_images_used": len(views),
        "rejected_views": [{"image": n, "error_px": round(e, 3)} for n, e in rejected],
        "per_view_error_px": {v[0].name: round(float(e), 3) for v, e in zip(views, per_view)},
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "opencv_version": cv2.__version__,
    }
    path = save_calibration(key, image_size, K, dist, rms, meta)
    check = PARAMS_DIR / f"{key}_check.jpg"
    write_check_image(check, views[int(np.argmin(per_view))][0], K, dist, image_size)
    print(f"\nSaved {path}\nCheck image: {check}")


if __name__ == "__main__":
    main()
