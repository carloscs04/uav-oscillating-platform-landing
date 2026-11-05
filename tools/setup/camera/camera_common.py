"""Shared setup for the scripts in tools/setup/camera. Not meant to be run directly.

Settings come from config/camera.yaml, so every script opens the camera the
same way. See README.md in this folder for the order to run the scripts in.
"""
import json
import sys
from pathlib import Path

import cv2
import yaml
from picamera2 import Picamera2

ROOT = Path(__file__).resolve().parents[3]
CONFIG_DIR = ROOT / "config"
SETTINGS_FILE = CONFIG_DIR / "camera.yaml"
STOCK_TUNING = Path("/usr/share/libcamera/ipa/rpi/pisp/imx219.json")

# Auto exposure needs longer frames in dim rooms than the flight frame rate allows.
AUTO_EXPOSURE_FPS = 30.0

# Fail rather than hang if the camera stops delivering frames.
FRAME_TIMEOUT_S = 5.0


def add_camera_args(parser):
    """Add options that override config/camera.yaml for a single run."""
    parser.add_argument("--exposure", type=int, help="exposure time in us")
    parser.add_argument("--gain", type=float, help="analogue gain")
    parser.add_argument("--fps", type=float, help="requested frame rate")
    parser.add_argument("--tuning", type=Path, help="tuning file to load")
    parser.add_argument("--stock", action="store_true", help="use the stock tuning")


def load_settings(args=None, auto_exposure=False):
    """Read config/camera.yaml and apply any command line overrides."""
    settings = yaml.safe_load(SETTINGS_FILE.read_text())
    settings["auto_exposure"] = auto_exposure
    if auto_exposure:
        settings["fps"] = AUTO_EXPOSURE_FPS
    if args is not None:
        if args.exposure is not None:
            settings["exposure_us"] = args.exposure
        if args.gain is not None:
            settings["gain"] = args.gain
        if args.fps is not None:
            settings["fps"] = args.fps
        if args.tuning is not None:
            settings["tuning_file"] = str(args.tuning.resolve())
        if args.stock:
            settings["tuning_file"] = None
    return settings


def tuning_path(settings):
    """Return the tuning file path, or None for the stock tuning."""
    name = settings["tuning_file"]
    return None if name is None else CONFIG_DIR / name


def make_tuning(path):
    """Write a copy of the stock tuning with lens shading correction removed."""
    tuning = json.loads(STOCK_TUNING.read_text())
    tuning["algorithms"] = [a for a in tuning["algorithms"] if "rpi.alsc" not in a]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(tuning, indent=4) + "\n")


def open_camera(settings):
    """Configure and start the camera. Exits with a clear message on failure."""
    tuning = None
    path = tuning_path(settings)
    if path is not None:
        if not path.exists():
            sys.exit(f"Tuning file not found: {path}\nRun tools/setup/camera/check_setup.py.")
        tuning = Picamera2.load_tuning_file(str(path))

    controls = {"FrameRate": settings["fps"]}
    if not settings["auto_exposure"]:
        controls.update(
            AeEnable=False,
            ExposureTime=settings["exposure_us"],
            AnalogueGain=settings["gain"],
        )

    try:
        picam2 = Picamera2(tuning=tuning)
    except RuntimeError:
        sys.exit(
            "Could not open the camera. Another program is probably using it.\n"
            "Find it with: pgrep -af 'live_view|capture_test|benchmark|rpicam'"
        )
    picam2.configure(picam2.create_video_configuration(
        raw={"size": tuple(settings["sensor_mode"])},
        main={"size": tuple(settings["output_size"]), "format": "RGB888"},
        controls=controls,
    ))
    picam2.start()
    return picam2


def grab(picam2):
    """Return the next frame and its metadata.

    RGB888 in Picamera2 is BGR byte order, so the frame is usable by OpenCV as is.
    Raises TimeoutError if the camera stops delivering frames.
    """
    request = picam2.capture_request(wait=FRAME_TIMEOUT_S)
    try:
        return request.make_array("main"), request.get_metadata()
    finally:
        request.release()


def build_detector(settings):
    name = settings["marker"]["dictionary"]
    dict_id = getattr(cv2.aruco, name, None)
    if dict_id is None:
        sys.exit(f"Unknown ArUco dictionary: {name}")
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())


def format_ids(ids):
    return "none" if ids is None else ",".join(map(str, sorted(ids.ravel().tolist())))


def annotate(frame, corners, ids, label):
    """Draw detected markers and a text label onto the frame."""
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.putText(frame, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
    cv2.putText(frame, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
