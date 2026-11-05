#!/usr/bin/env python3
"""Step 7 of 7: benchmark the vision pipeline against the scope targets.

Runs with the settings in config/camera.yaml and reports delivered FPS, frame
jitter and dropped frames (from sensor timestamps), the cost of each stage, and
the marker detection rate. Keep the marker moving in view during the run.
Headless. Do not run with sudo.

    .venv/bin/python tools/setup/camera/benchmark.py --seconds 30
    .venv/bin/python tools/setup/camera/benchmark.py --capture-only

Pose estimation is not included yet, so this is a partial pipeline for now.
"""
import argparse
import sys
import time

import cv2
import numpy as np

from camera_common import add_camera_args, build_detector, grab, load_settings, open_camera


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    add_camera_args(p)
    p.add_argument("--seconds", type=float, default=10.0, help="measurement length")
    p.add_argument("--warmup", type=float, default=1.0, help="discarded startup time")
    p.add_argument("--auto-exposure", action="store_true", help="leave auto exposure on")
    p.add_argument("--capture-only", action="store_true", help="skip ArUco detection")
    p.add_argument("--save", metavar="PATH", help="write the last frame to this image file")
    return p.parse_args()


def stats_ms(values):
    a = np.asarray(values, dtype=float)
    return f"mean {a.mean():6.2f}  p95 {np.percentile(a, 95):6.2f}  max {a.max():6.2f} ms"


def main():
    args = parse_args()
    settings = load_settings(args, auto_exposure=args.auto_exposure)
    detector = None if args.capture_only else build_detector(settings)
    picam2 = open_camera(settings)

    sensor_ts, grab_ms, gray_ms, detect_ms = [], [], [], []
    frames_with_marker = 0
    frame = None
    metadata = {}

    try:
        warmup_end = time.perf_counter() + args.warmup
        while time.perf_counter() < warmup_end:
            grab(picam2)

        start = time.perf_counter()
        end = start + args.seconds
        while time.perf_counter() < end:
            t0 = time.perf_counter()
            frame, metadata = grab(picam2)
            t1 = time.perf_counter()
            grab_ms.append((t1 - t0) * 1e3)
            sensor_ts.append(metadata["SensorTimestamp"])

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            t2 = time.perf_counter()
            gray_ms.append((t2 - t1) * 1e3)

            if detector is not None:
                _, ids, _ = detector.detectMarkers(gray)
                detect_ms.append((time.perf_counter() - t2) * 1e3)
                frames_with_marker += ids is not None
        elapsed = time.perf_counter() - start
    finally:
        picam2.stop()
        picam2.close()

    n = len(sensor_ts)
    if n < 2:
        sys.exit("Too few frames captured to report anything.")

    fps = n / elapsed
    intervals_ms = np.diff(np.asarray(sensor_ts, dtype=np.int64)) / 1e6
    nominal_ms = float(np.median(intervals_ms))
    dropped = int(np.sum(np.round(intervals_ms / nominal_ms).astype(int) - 1))

    print(f"Stream        : {frame.shape[1]}x{frame.shape[0]}, "
          f"crop {metadata.get('ScalerCrop')}")
    print(f"Tuning        : {settings['tuning_file'] or 'stock'}")
    print(f"Requested     : {settings['fps']:.1f} fps")
    print(f"Exposure/gain : {metadata.get('ExposureTime')} us / "
          f"{metadata.get('AnalogueGain', 0):.2f}")
    print(f"Mean luma     : {cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean():.1f} / 255")
    print(f"Frames        : {n} in {elapsed:.2f} s")
    print(f"Delivered FPS : {fps:.2f}")
    print(f"Sensor period : {stats_ms(intervals_ms)}")
    print(f"Dropped       : {dropped} ({100 * dropped / (n + dropped):.1f}%)")
    print(f"Wait + grab   : {stats_ms(grab_ms)}")
    print(f"Grayscale     : {stats_ms(gray_ms)}")

    if args.save:
        if not cv2.imwrite(args.save, frame):
            sys.exit(f"Could not write {args.save}")
        print(f"Saved         : {args.save}")

    targets = settings["targets"]
    fps_ok = fps >= targets["min_fps"]
    print(f"FPS target    : {'PASS' if fps_ok else 'FAIL'} (>= {targets['min_fps']} FPS)")
    if detector is None:
        return 0 if fps_ok else 1

    rate = frames_with_marker / n
    detect_ok = rate >= targets["min_detection"]
    print(f"ArUco detect  : {stats_ms(detect_ms)}")
    print(f"Marker frames : {frames_with_marker}/{n} ({100 * rate:.1f}%)")
    print(f"Detection     : {'PASS' if detect_ok else 'FAIL'} "
          f"(>= {100 * targets['min_detection']:.0f}%)")
    return 0 if fps_ok and detect_ok else 1


if __name__ == "__main__":
    sys.exit(main())
