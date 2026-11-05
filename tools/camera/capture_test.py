#!/usr/bin/env python3
"""Headless capture test with the camera settings.

Opens the IMX219 in full-FOV mode with the no-ALSC tuning file and manual
exposure, detects ArUco markers, prints FPS and IDs once a second, and saves a
few annotated frames. Never opens a window. Do not run with sudo.

    python3 tools/camera/capture_test.py --make-tuning    # first run: create tuning file
    python3 tools/camera/capture_test.py --seconds 20 --save-count 8

If the image is too dark, add light to the flight cage. A longer exposure
brings back motion blur.
"""
import argparse
import json
import sys
import time
from collections import Counter
from pathlib import Path

import cv2
from picamera2 import Picamera2

ROOT = Path(__file__).resolve().parents[2]
STOCK_TUNING = Path("/usr/share/libcamera/ipa/rpi/pisp/imx219.json")
TUNING = ROOT / "config" / "imx219_noalsc.json"

SENSOR_MODE = (1640, 1232)  # full sensor, binned -> full FOV
MAIN_SIZE = (640, 480)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--seconds", type=float, default=10.0, help="test length")
    p.add_argument("--exposure", type=int, default=4000, help="exposure time in us")
    p.add_argument("--gain", type=float, default=6.0, help="analogue gain")
    p.add_argument("--fps", type=float, default=60.0, help="requested frame rate")
    p.add_argument("--save-count", type=int, default=5, help="annotated frames to save")
    p.add_argument("--out", type=Path, default=ROOT / "captures", help="output directory")
    p.add_argument("--tuning", type=Path, default=TUNING, help="tuning file to load")
    p.add_argument("--make-tuning", action="store_true",
                   help="create the no-ALSC tuning file if it does not exist")
    return p.parse_args()


def make_tuning(path):
    """Write a copy of the stock tuning with lens shading correction removed."""
    if path.exists():
        print(f"Tuning file already exists: {path}")
        return
    tuning = json.loads(STOCK_TUNING.read_text())
    tuning["algorithms"] = [a for a in tuning["algorithms"] if "rpi.alsc" not in a]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(tuning, indent=4) + "\n")
    print(f"Created tuning file: {path}")


def open_camera(args):
    picam2 = Picamera2(tuning=Picamera2.load_tuning_file(str(args.tuning.resolve())))
    picam2.configure(picam2.create_video_configuration(
        raw={"size": SENSOR_MODE},
        main={"size": MAIN_SIZE, "format": "RGB888"},
        controls={
            "AeEnable": False,
            "ExposureTime": args.exposure,
            "AnalogueGain": args.gain,
            "FrameRate": args.fps,
        },
    ))
    picam2.start()
    return picam2


def save_annotated(frame, corners, ids, out_dir, index, elapsed):
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    found = "none" if ids is None else ",".join(map(str, sorted(ids.ravel().tolist())))
    label = f"t={elapsed:.1f}s markers: {found}"
    cv2.putText(frame, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
    cv2.putText(frame, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    path = out_dir / f"frame_{index:02d}.jpg"
    if not cv2.imwrite(str(path), frame):
        sys.exit(f"Could not write {path}")
    print(f"Saved {path}")


def main():
    args = parse_args()

    if args.make_tuning:
        make_tuning(args.tuning)
    if not args.tuning.exists():
        sys.exit(f"Tuning file not found: {args.tuning}\nRun again with --make-tuning.")
    args.out.mkdir(parents=True, exist_ok=True)

    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters(),
    )

    picam2 = open_camera(args)
    frames = frames_with_marker = saved = 0
    seen = Counter()
    metadata = {}
    save_interval = args.seconds / max(args.save_count, 1)
    window_frames, window_ids = 0, set()

    try:
        start = window_start = time.perf_counter()
        while (elapsed := time.perf_counter() - start) < args.seconds:
            request = picam2.capture_request()
            try:
                # RGB888 in Picamera2 is BGR byte order: usable by OpenCV as is.
                frame = request.make_array("main")
                metadata = request.get_metadata()
            finally:
                request.release()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            frames += 1
            window_frames += 1
            if ids is not None:
                frames_with_marker += 1
                seen.update(ids.ravel().tolist())
                window_ids.update(ids.ravel().tolist())

            if saved < args.save_count and elapsed >= saved * save_interval:
                saved += 1
                save_annotated(frame, corners, ids, args.out, saved, elapsed)

            now = time.perf_counter()
            if now - window_start >= 1.0:
                fps = window_frames / (now - window_start)
                print(f"{fps:5.1f} fps | IDs: {sorted(window_ids) or 'none'}", flush=True)
                window_start, window_frames, window_ids = now, 0, set()
        total = time.perf_counter() - start
    except KeyboardInterrupt:
        total = time.perf_counter() - start
    finally:
        picam2.stop()
        picam2.close()

    print("--- Summary ---")
    print(f"Frames        : {frames} in {total:.2f} s ({frames / total:.1f} fps)")
    print(f"Exposure/gain : {metadata.get('ExposureTime')} us / "
          f"{metadata.get('AnalogueGain', 0):.2f}")
    if frames:
        print(f"Marker frames : {frames_with_marker}/{frames} "
              f"({100 * frames_with_marker / frames:.1f}%)")
    print(f"IDs seen      : {dict(sorted(seen.items())) or 'none'}")
    print(f"Saved frames  : {saved} in {args.out}")


if __name__ == "__main__":
    main()
