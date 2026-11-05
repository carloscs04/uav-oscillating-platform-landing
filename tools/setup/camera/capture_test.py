#!/usr/bin/env python3
"""Step 3 of 7: test exposure and gain with the marker in view.

Runs with the settings in config/camera.yaml, detects ArUco markers, prints FPS
and IDs once a second, and saves a few annotated frames. Move the marker during
the run, ideally under flight cage lighting. Headless. Do not run with sudo.

    .venv/bin/python tools/setup/camera/capture_test.py --seconds 20 --save-count 8
    .venv/bin/python tools/setup/camera/capture_test.py --exposure 3000 --gain 8

Once a pair of values passes, save it in config/camera.yaml. If the image is too
dark, add light to the flight cage: a longer exposure brings back motion blur.

Next: step 4, ChArUco image capture (not built yet).
"""
import argparse
import sys
import time
from collections import Counter
from pathlib import Path

import cv2

from camera_common import (
    ROOT, add_camera_args, annotate, build_detector, format_ids, grab, load_settings,
    open_camera,
)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    add_camera_args(p)
    p.add_argument("--seconds", type=float, default=10.0, help="test length")
    p.add_argument("--save-count", type=int, default=5, help="annotated frames to save")
    p.add_argument("--out", type=Path, default=ROOT / "captures", help="output directory")
    return p.parse_args()


def save_annotated(frame, corners, ids, out_dir, index, elapsed):
    annotate(frame, corners, ids, f"t={elapsed:.1f}s markers: {format_ids(ids)}")
    path = out_dir / f"frame_{index:02d}.jpg"
    if not cv2.imwrite(str(path), frame):
        sys.exit(f"Could not write {path}")
    print(f"Saved {path}")


def main():
    args = parse_args()
    settings = load_settings(args)
    args.out.mkdir(parents=True, exist_ok=True)
    detector = build_detector(settings)
    picam2 = open_camera(settings)

    frames = frames_with_marker = saved = 0
    seen = Counter()
    metadata = {}
    save_interval = args.seconds / max(args.save_count, 1)
    window_frames, window_ids = 0, set()

    try:
        start = window_start = time.perf_counter()
        while (elapsed := time.perf_counter() - start) < args.seconds:
            frame, metadata = grab(picam2)
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
    except KeyboardInterrupt:
        pass
    finally:
        total = time.perf_counter() - start
        picam2.stop()
        picam2.close()

    if not frames:
        sys.exit("No frames captured.")

    rate = frames_with_marker / frames
    target = settings["targets"]["min_detection"]
    print("--- Summary ---")
    print(f"Frames        : {frames} in {total:.2f} s ({frames / total:.1f} fps)")
    print(f"Exposure/gain : {metadata.get('ExposureTime')} us / "
          f"{metadata.get('AnalogueGain', 0):.2f}")
    print(f"Marker frames : {frames_with_marker}/{frames} ({100 * rate:.1f}%)")
    print(f"IDs seen      : {dict(sorted(seen.items())) or 'none'}")
    print(f"Saved frames  : {saved} in {args.out}")
    print(f"Result        : {'PASS' if rate >= target else 'FAIL'} "
          f"(detection >= {100 * target:.0f}%)")
    return 0 if rate >= target else 1


if __name__ == "__main__":
    sys.exit(main())
