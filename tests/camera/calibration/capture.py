"""
capture.py - collect ChArUco calibration images from one camera.

    python3 -m calibration.capture                       # active_camera from config.yaml
    python3 -m calibration.capture --camera usb_0bdc_8088
    python3 -m calibration.capture --auto                # start with auto-save on
    python3 -m calibration.capture --clear               # start over (old set is kept, renamed)

Images go to calibration/images/<calib_key>/img_000.png ... (raw frames, no overlay).
The live view opens a window on the Pi desktop, or over SSH a browser stream
at http://<pi-ip>:8000/ (see viewer.py).

Keys:  s = save now   a = auto-save on/off   d = delete last image   q = quit

How to get a good calibration (aim for ~30 images):
  * hold the board STILL when saving (motion blur ruins corners)
  * fill the green coverage grid: board near every edge and corner of the image
  * tilt the board ~20-45 degrees left/right/up/down in a third of the images
  * vary the distance: board filling most of the frame, and about half of it
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

if __package__ in (None, ""):  # allow `python3 calibration/capture.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import _paths  # noqa: E402,F401  (camera layer lives in src/inputs/camera_driver)

from calibration.board import (board_config, describe, detect_corners,  # noqa: E402
                               make_board, make_detector, n_corners)
from calibration.params import images_dir  # noqa: E402
from cameras import load_config, open_camera  # noqa: E402
from viewer import make_viewer  # noqa: E402

GRID = (8, 6)              # coverage grid cells (x, y)
MAX_MOTION_PX = 1.5        # corner movement between frames to count as "still"
MIN_NOVELTY = 0.15         # how different a pose must be from saved ones for auto-save
AUTO_COOLDOWN_S = 0.7      # minimum time between auto-saves
TILT_DEG = 20              # a view counts as "tilted" above this


class PoseTracker:
    """Rough board pose per view, to tell new views from repeated ones.

    Uses a guessed focal length (the camera is not calibrated yet), which is
    accurate enough to compare views with each other.
    """

    def __init__(self, board, image_size):
        w, h = image_size
        self.board = board
        self.size = image_size
        self.K = np.array([[w, 0, w / 2], [0, w, h / 2], [0, 0, 1]], dtype=np.float64)

    def feature(self, corners, ids):
        """[centre_x, centre_y, scale, tilt_x, tilt_y], roughly 0..1 ranges."""
        w, h = self.size
        pts = corners.reshape(-1, 2)
        cx, cy = pts.mean(axis=0) / (w, h)
        hull_area = cv2.contourArea(cv2.convexHull(pts.astype(np.float32)))
        scale = np.sqrt(hull_area / (w * h))
        tilt_x = tilt_y = 0.0
        obj, img = self.board.matchImagePoints(corners, ids)
        if obj is not None and len(obj) >= 6:
            ok, rvec, _ = cv2.solvePnP(obj, img, self.K, None)
            if ok:
                R, _ = cv2.Rodrigues(rvec)
                normal = R[:, 2]  # board normal in camera frame
                tilt_x = np.degrees(np.arctan2(normal[0], -normal[2]))
                tilt_y = np.degrees(np.arctan2(normal[1], -normal[2]))
        return np.array([cx, cy, scale, tilt_x / 90.0, tilt_y / 90.0])

    @staticmethod
    def is_tilted(feature):
        return max(abs(feature[3]), abs(feature[4])) * 90.0 > TILT_DEG


class Coverage:
    def __init__(self, image_size):
        self.size = image_size
        self.counts = np.zeros((GRID[1], GRID[0]), dtype=int)

    def _cells(self, corners):
        w, h = self.size
        pts = corners.reshape(-1, 2)
        gx = np.clip((pts[:, 0] / w * GRID[0]).astype(int), 0, GRID[0] - 1)
        gy = np.clip((pts[:, 1] / h * GRID[1]).astype(int), 0, GRID[1] - 1)
        return set(zip(gy, gx))

    def add(self, corners, sign=1):
        for cell in self._cells(corners):
            self.counts[cell] += sign

    @property
    def percent(self):
        return 100.0 * np.count_nonzero(self.counts) / self.counts.size

    def draw(self, img):
        h, w = img.shape[:2]
        overlay = img.copy()
        for gy in range(GRID[1]):
            for gx in range(GRID[0]):
                if self.counts[gy, gx]:
                    p0 = (gx * w // GRID[0], gy * h // GRID[1])
                    p1 = ((gx + 1) * w // GRID[0], (gy + 1) * h // GRID[1])
                    cv2.rectangle(overlay, p0, p1, (0, 200, 0), -1)
        cv2.addWeighted(overlay, 0.25, img, 0.75, 0, dst=img)
        for gx in range(1, GRID[0]):
            cv2.line(img, (gx * w // GRID[0], 0), (gx * w // GRID[0], h), (90, 90, 90), 1)
        for gy in range(1, GRID[1]):
            cv2.line(img, (0, gy * h // GRID[1]), (w, gy * h // GRID[1]), (90, 90, 90), 1)


def motion_px(prev, corners, ids):
    """Mean movement of corners seen in both frames, or inf if not comparable."""
    if prev is None or corners is None:
        return float("inf")
    prev_map = dict(zip(prev[1].ravel().tolist(), prev[0].reshape(-1, 2)))
    d = [np.linalg.norm(p - prev_map[i])
         for i, p in zip(ids.ravel().tolist(), corners.reshape(-1, 2)) if i in prev_map]
    return float(np.mean(d)) if len(d) >= 4 else float("inf")


def put_lines(img, lines, origin=(10, 30)):
    scale = max(0.6, img.shape[1] / 1600)
    y = origin[1]
    for text, color in lines:
        cv2.putText(img, text, (origin[0], y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), int(5 * scale), cv2.LINE_AA)
        cv2.putText(img, text, (origin[0], y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color, int(2 * scale), cv2.LINE_AA)
        y += int(34 * scale)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", help="camera name from config.yaml (default: active_camera)")
    ap.add_argument("--config", help="path to config.yaml")
    ap.add_argument("--auto", action="store_true", help="start with auto-save on")
    ap.add_argument("--clear", action="store_true",
                    help="start a new image set; the old folder is renamed, not deleted")
    ap.add_argument("--view", choices=["auto", "window", "web"], default="auto")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()

    config = load_config(args.config)
    bcfg = board_config(config)
    ccfg = config.get("calibration", {})
    min_corners = int(ccfg.get("min_corners", 12))
    target = int(ccfg.get("target_images", 30))
    total_corners = n_corners(bcfg)

    board = make_board(bcfg)
    detector = make_detector(board)

    # Calibration must target one specific camera: never fall back to another.
    cam = open_camera(args.camera, config, fallback=False)
    out = images_dir(cam.calib_key)
    if args.clear and out.exists() and any(out.iterdir()):
        old = out.with_name(f"{out.name}_old_{time.strftime('%Y%m%d_%H%M%S')}")
        out.rename(old)
        print(f"Previous images moved to {old}")
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "camera.yaml", "w") as f:
        yaml.safe_dump({"camera": cam.info(), "board": dict(bcfg)}, f, sort_keys=False)

    coverage = Coverage(cam.resolution)
    poses = PoseTracker(board, cam.resolution)
    saved = []  # list of (path, corners, feature)

    # Resume: include images already in the folder in coverage/novelty.
    for path in sorted(out.glob("img_*.png")):
        corners, ids = detect_corners(detector, cv2.imread(str(path), cv2.IMREAD_GRAYSCALE))
        if corners is not None and len(ids) >= min_corners:
            coverage.add(corners)
            saved.append((path, corners, poses.feature(corners, ids)))
    next_index = 1 + max([int(p.stem.split("_")[1]) for p in out.glob("img_*.png")] or [-1])

    print(f"Camera: {cam.calib_key}   Board: {describe(bcfg)}")
    print(f"Saving to {out}  ({len(saved)} usable images already there)")
    viewer = make_viewer(f"calibration capture - {cam.calib_key}", mode=args.view,
                         port=args.port, keys={"s": "save", "a": "auto on/off",
                                               "d": "delete last", "q": "quit"})

    auto = args.auto
    prev = None
    last_save_t, flash = 0.0, 0

    def save(frame, corners, ids, feature):
        nonlocal next_index, last_save_t, flash
        path = out / f"img_{next_index:03d}.png"
        cv2.imwrite(str(path), frame)
        next_index += 1
        coverage.add(corners)
        saved.append((path, corners, feature))
        last_save_t, flash = time.monotonic(), 4
        print(f"Saved {path.name}  ({len(saved)}/{target}, {len(corners)} corners, "
              f"coverage {coverage.percent:.0f}%)")

    try:
        while True:
            ok, frame, _ = cam.read()
            if not ok:
                print("frame grab failed")
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids = detect_corners(detector, gray)
            n = 0 if ids is None else len(ids)
            motion = motion_px(prev, corners, ids)
            prev = (corners, ids) if corners is not None else None

            feature, novelty = None, 0.0
            if n >= min_corners:
                feature = poses.feature(corners, ids)
                novelty = min([np.linalg.norm(feature - s[2]) for s in saved] or [1.0])

            still = motion < MAX_MOTION_PX
            if n < min_corners:
                status, color = f"board not found / too few corners ({n}/{total_corners})", (0, 0, 255)
            elif not still:
                status, color = "hold still...", (0, 200, 255)
            elif novelty < MIN_NOVELTY:
                status, color = "pose too similar to a saved one - move/tilt the board", (0, 200, 255)
            else:
                status, color = "READY", (0, 255, 0)

            if (auto and status == "READY"
                    and time.monotonic() - last_save_t > AUTO_COOLDOWN_S):
                save(frame, corners, ids, feature)

            # ---- draw
            view = frame.copy()
            coverage.draw(view)
            if corners is not None:
                cv2.aruco.drawDetectedCornersCharuco(view, corners, ids, (0, 255, 0))
            if flash:
                cv2.rectangle(view, (0, 0), (view.shape[1] - 1, view.shape[0] - 1),
                              (255, 255, 255), 25)
                flash -= 1
            tilted = sum(PoseTracker.is_tilted(s[2]) for s in saved)
            put_lines(view, [
                (f"{cam.calib_key}   images {len(saved)}/{target}   "
                 f"coverage {coverage.percent:.0f}%   tilted {tilted}/{max(1, target // 3)}",
                 (255, 255, 255)),
                (f"corners {n}/{total_corners}   auto-save {'ON' if auto else 'off'}",
                 (255, 255, 255)),
                (status, color),
            ])

            key = viewer.show(view)
            if key == "q":
                break
            elif key == "a":
                auto = not auto
                print(f"auto-save {'ON' if auto else 'off'}")
            elif key == "s":
                if n >= min_corners:
                    if not still:
                        print("warning: board was moving, image may be blurred")
                    save(frame, corners, ids, feature)
                else:
                    print(f"not saved: only {n} corners (need {min_corners})")
            elif key == "d" and saved:
                path, c, _ = saved.pop()
                coverage.add(c, sign=-1)
                path.unlink(missing_ok=True)
                print(f"Deleted {path.name}  ({len(saved)}/{target})")
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()
        cam.close()

    print(f"\n{len(saved)} images in {out}, coverage {coverage.percent:.0f}%")
    if len(saved) < 15:
        print("Fewer than 15 images: calibration will be unreliable, capture more.")
    print(f"Next: python3 -m calibration.calibrate --camera {cam.camera_id}")


if __name__ == "__main__":
    main()
