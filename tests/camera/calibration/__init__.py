"""Camera calibration with a ChArUco board.

Run from tests/camera:
    python3 -m calibration.board               # 1. make the printable board
    python3 -m calibration.capture             # 2. collect images (active_camera)
    python3 -m calibration.calibrate           # 3. compute + save the result

Images:  tests/camera/calibration/images/<calib_key>/
Results: src/inputs/camera_driver/calib/<calib_key>.yaml  (used on the drone)

Other code loads a result with:
    from calibration import load_calibration
    calib = load_calibration(cam.calib_key)
"""

import sys
from pathlib import Path

# The camera layer and calibration storage live in src/inputs/camera_driver.
_TOOLS = str(Path(__file__).resolve().parent.parent)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import _paths  # noqa: E402,F401

from .params import (IMAGES_DIR, PARAMS_DIR, CalibrationNotFound,  # noqa: E402
                     images_dir, load_calibration, params_path)

__all__ = ["IMAGES_DIR", "PARAMS_DIR", "CalibrationNotFound",
           "images_dir", "load_calibration", "params_path"]
