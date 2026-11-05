#!/usr/bin/env python3
"""Measure onboard camera throughput with Picamera2 + OpenCV.

Runs the IMX219 in its full-FOV binned mode scaled to 640x480 and reports:
  - delivered FPS, frame-interval jitter and dropped frames (from sensor timestamps)
  - per-stage cost of grayscale conversion and ArUco detection

Headless-safe: never opens a window. Do not run with sudo.

    python3 tools/camera/capture_fps.py                  # capture only
    python3 tools/camera/capture_fps.py --aruco          # capture + detection
    python3 tools/camera/capture_fps.py --save frame.png
"""
import argparse
import sys
import time

import cv2
import numpy as np
from picamera2 import Picamera2

SENSOR_MODE = (1640, 1232)  # full sensor, binned -> full FOV
TARGET_FPS = 15.0           # scope statement: vision pipeline >= 15 FPS onboard


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--seconds", type=float, default=10.0, help="measurement length")
    p.add_argument("--warmup", type=float, default=1.0, help="discarded startup time")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=float, default=60.0, help="requested sensor frame rate")
    p.add_argument("--exposure", type=int, default=4000, help="exposure time in us")
    p.add_argument("--gain", type=float, default=4.0, help="analogue gain")
    p.add_argument("--auto-exposure", action="store_true", help="leave AE enabled")
    p.add_argument("--aruco", action="store_true", help="run ArUco detection on each frame")
    p.add_argument("--dict", default="DICT_4X4_50", help="cv2.aruco dictionary name")
    p.add_argument("--save", metavar="PATH", help="write the last frame to this image file")
    return p.parse_args()


def build_controls(args):
    frame_us = int(1_000_000 / args.fps)
    controls = {"FrameDurationLimits": (frame_us, frame_us)}
    if args.auto_exposure:
        controls["AeEnable"] = True
    else:
        controls.update(
            AeEnable=False, ExposureTime=args.exposure, AnalogueGain=args.gain
        )
    return controls


def build_detector(name):
    dict_id = getattr(cv2.aruco, name, None)
    if dict_id is None:
        sys.exit(f"Unknown ArUco dictionary: {name}")
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())


def stats_ms(values):
    a = np.asarray(values, dtype=float)
    return f"mean {a.mean():6.2f}  p95 {np.percentile(a, 95):6.2f}  max {a.max():6.2f} ms"


def main():
    args = parse_args()
    detector = build_detector(args.dict) if args.aruco else None

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        raw={"size": SENSOR_MODE},
        main={"size": (args.width, args.height), "format": "RGB888"},
        controls=build_controls(args),
    )
    picam2.configure(config)
    sensor = picam2.camera_configuration()["raw"]
    picam2.start()

    sensor_ts, grab_ms, gray_ms, detect_ms = [], [], [], []
    frames_with_marker = 0
    frame = None
    metadata = {}

    try:
        warmup_end = time.perf_counter() + args.warmup
        while time.perf_counter() < warmup_end:
            picam2.capture_request().release()

        start = time.perf_counter()
        end = start + args.seconds
        while time.perf_counter() < end:
            t0 = time.perf_counter()
            request = picam2.capture_request()
            try:
                # RGB888 in Picamera2 is BGR byte order: usable by OpenCV as is.
                frame = request.make_array("main")
                metadata = request.get_metadata()
            finally:
                request.release()
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
          f"sensor {sensor['size']} {sensor['format']}")
    print(f"Scaler crop   : {metadata.get('ScalerCrop')} (x, y, w, h on the 3280x2464 array)")
    print(f"Requested     : {args.fps:.1f} fps")
    print(f"Exposure/gain : {metadata.get('ExposureTime')} us / "
          f"{metadata.get('AnalogueGain', 0):.2f}")
    print(f"Mean luma     : {cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean():.1f} / 255")
    print(f"Frames        : {n} in {elapsed:.2f} s")
    print(f"Delivered FPS : {fps:.2f}")
    print(f"Sensor period : {stats_ms(intervals_ms)}")
    print(f"Dropped       : {dropped} ({100 * dropped / (n + dropped):.1f}%)")
    print(f"Wait + grab   : {stats_ms(grab_ms)}")
    print(f"Grayscale     : {stats_ms(gray_ms)}")
    if detector is not None:
        print(f"ArUco detect  : {stats_ms(detect_ms)}")
        print(f"Marker frames : {frames_with_marker}/{n} ({args.dict})")

    if args.save:
        if not cv2.imwrite(args.save, frame):
            sys.exit(f"Could not write {args.save}")
        print(f"Saved         : {args.save}")

    ok = fps >= TARGET_FPS
    print(f"Target        : {'PASS' if ok else 'FAIL'} (>= {TARGET_FPS:.0f} FPS)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
