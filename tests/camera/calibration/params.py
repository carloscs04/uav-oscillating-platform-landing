"""Calibration file locations for the tools.

Results (params) live with the drone code in src/inputs/camera_driver/calib/
(see calib_params.py there). The captured images are development data and
stay here, in tests/camera/calibration/images/<calib_key>/.
"""

from pathlib import Path

from calib_params import (PARAMS_DIR, Calibration, CalibrationNotFound,  # noqa: F401
                          load_calibration, params_path, save_calibration)

IMAGES_DIR = Path(__file__).resolve().parent / "images"


def images_dir(calib_key):
    return IMAGES_DIR / calib_key
