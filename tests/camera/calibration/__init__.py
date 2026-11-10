"""Camera calibration with a ChArUco board.

Run from tests/camera:
    python3 -m calibration.board               # 1. make the printable board
    python3 -m calibration.capture             # 2. collect images (active_camera)
    python3 -m calibration.calibrate           # 3. compute + save params/<calib_key>.yaml

Other code loads the result with:
    from calibration import load_calibration
    calib = load_calibration(cam.calib_key)
"""

from .params import (IMAGES_DIR, PARAMS_DIR, CalibrationNotFound,
                     load_calibration, params_path)

__all__ = ["IMAGES_DIR", "PARAMS_DIR", "CalibrationNotFound",
           "load_calibration", "params_path"]
