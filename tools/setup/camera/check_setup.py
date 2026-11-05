#!/usr/bin/env python3
"""Step 1 of 7: check the Pi and camera are ready.

Checks user permissions, settings, OpenCV, the tuning file (creating it if
missing), that the camera delivers a frame, and temperature. Do not run with
sudo, and stop any other camera script first.

    .venv/bin/python tools/setup/camera/check_setup.py

Next: step 2, live_view.py.
"""
import grp
import os
import subprocess
import sys

import cv2

from camera_common import SETTINGS_FILE, grab, load_settings, make_tuning, open_camera, tuning_path

REQUIRED_GROUPS = {"video", "render"}
MAX_TEMP_C = 80.0


def check_user():
    if os.geteuid() == 0:
        return False, "running as root; vision scripts must not use sudo"
    groups = {grp.getgrgid(gid).gr_name for gid in os.getgroups()}
    missing = REQUIRED_GROUPS - groups
    if missing:
        return False, f"missing groups: {', '.join(sorted(missing))}"
    return True, f"in groups {', '.join(sorted(REQUIRED_GROUPS))}"


def check_settings():
    settings = load_settings()
    return True, (f"{SETTINGS_FILE.name}: {settings['exposure_us']} us, "
                  f"gain {settings['gain']}, {settings['fps']} fps")


def check_opencv():
    if not hasattr(cv2.aruco, "ArucoDetector"):
        return False, f"OpenCV {cv2.__version__} lacks ArucoDetector (needs >= 4.7)"
    return True, f"OpenCV {cv2.__version__}"


def check_tuning():
    path = tuning_path(load_settings())
    if path is None:
        return True, "stock tuning"
    if path.exists():
        return True, path.name
    make_tuning(path)
    return True, f"created {path.name}"


def check_camera():
    settings = load_settings()
    try:
        picam2 = open_camera(settings)
    except SystemExit as error:
        return False, str(error)
    try:
        frame, metadata = grab(picam2)
    finally:
        picam2.stop()
        picam2.close()
    return True, (f"{frame.shape[1]}x{frame.shape[0]} frame, "
                  f"crop {metadata.get('ScalerCrop')}")


def check_temperature():
    def vcgencmd(*command):
        return subprocess.run(
            ["vcgencmd", *command], capture_output=True, text=True, check=True
        ).stdout.strip()

    temp = float(vcgencmd("measure_temp").split("=")[1].split("'")[0])
    throttled = vcgencmd("get_throttled").split("=")[1]
    ok = temp < MAX_TEMP_C and throttled == "0x0"
    return ok, f"{temp:.1f} C, throttle flags {throttled}"


CHECKS = [
    ("User", check_user),
    ("Settings", check_settings),
    ("OpenCV", check_opencv),
    ("Tuning file", check_tuning),
    ("Camera", check_camera),
    ("Temperature", check_temperature),
]


def main():
    all_ok = True
    for name, check in CHECKS:
        try:
            ok, detail = check()
        except Exception as error:
            ok, detail = False, f"{type(error).__name__}: {error}"
        all_ok = all_ok and ok
        print(f"[{'PASS' if ok else 'FAIL'}] {name:12s}: {detail}", flush=True)

    print("Result: PASS. Next: live_view.py" if all_ok else "Result: FAIL. Fix the items above.")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
