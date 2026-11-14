"""
preview.py - open a camera through the camera layer and check it works.

Usage (from tests/camera):
    python3 preview.py                       # active_camera from config.yaml, live view
    python3 preview.py --camera usb_0bdc_8088
    python3 preview.py --headless --frames 60 --save   # no view: measure FPS, save a frame

The live view opens a window on the Pi desktop, or over SSH streams to
http://<pi-ip>:8000/ (force one with --view window|web, change port with --port).

Keys:  s = save frame to captures/,  g = toggle black & white,  q / Esc = quit
(--gray starts in black & white; saved frames are always the original colour frame)
"""

import argparse
import logging
import os
import time

import cv2

import _paths  # noqa: F401  (camera layer lives in src/inputs/camera_driver)
from cameras import open_camera
from viewer import make_viewer

# Saved frames go next to this script, wherever it is run from.
CAPTURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")


def _capture_path(cam):
    os.makedirs(CAPTURE_DIR, exist_ok=True)  # created only when something is saved
    return os.path.join(CAPTURE_DIR, f"{cam.calib_key}_{int(time.time())}.png")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", help="camera name from config.yaml (default: active_camera)")
    ap.add_argument("--config", help="path to config.yaml")
    ap.add_argument("--headless", action="store_true", help="no window; print stats only")
    ap.add_argument("--frames", type=int, default=90, help="frames to grab in headless mode")
    ap.add_argument("--save", action="store_true", help="headless: save the last frame")
    ap.add_argument("--no-fallback", action="store_true", help="fail instead of trying other cameras")
    ap.add_argument("--view", choices=["auto", "window", "web"], default="auto",
                    help="live view type (auto: window if a display exists, else web)")
    ap.add_argument("--port", type=int, default=8000, help="port for the web view")
    ap.add_argument("--gray", action="store_true", help="start the live view in black & white")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    with open_camera(args.camera, args.config,
                     fallback=False if args.no_fallback else None) as cam:
        print(f"Opened {cam}")
        for k, v in cam.info().items():
            print(f"  {k}: {v}")

        if args.headless:
            run_headless(cam, args.frames, args.save)
        else:
            run_live(cam, args.view, args.port, args.gray)


def run_headless(cam, n_frames, save):
    t0 = time.monotonic()
    frame, ok_count = None, 0
    for _ in range(n_frames):
        ok, f, _ = cam.read()
        if ok:
            ok_count, frame = ok_count + 1, f
    dt = time.monotonic() - t0
    print(f"{ok_count}/{n_frames} frames in {dt:.2f}s -> {ok_count / dt:.1f} FPS")
    if save and frame is not None:
        path = _capture_path(cam)
        cv2.imwrite(path, frame)
        print(f"Saved {path}")


def run_live(cam, mode, port, gray=False):
    viewer = make_viewer(f"preview - {cam.calib_key}", mode=mode, port=port,
                         keys={"s": "save frame", "g": "black & white", "q": "quit"})
    last, fps = time.monotonic(), 0.0
    try:
        while True:
            ok, frame, stamp = cam.read()
            if not ok:
                print("frame grab failed")
                break
            fps = 0.9 * fps + 0.1 / max(stamp - last, 1e-6)
            last = stamp
            if gray:
                # Back to 3 channels so the green overlay text still draws.
                view = cv2.cvtColor(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
                                    cv2.COLOR_GRAY2BGR)
            else:
                view = frame.copy()
            cv2.putText(view, f"{cam.calib_key}  {fps:.1f} FPS", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            key = viewer.show(view)
            if key == "q":
                break
            if key == "g":
                gray = not gray
            if key == "s":
                path = _capture_path(cam)
                cv2.imwrite(path, frame)  # save the clean frame, not the overlay
                print(f"Saved {path}")
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
