"""Make the drone code importable from the tools in tests/camera.

The camera layer (`cameras`), `config.yaml` and the calibration results live
with the drone code in src/inputs/camera_driver/. That folder is put on
sys.path so the tools keep their short imports:
    from cameras import open_camera
    from calib_params import load_calibration
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CAMERA_DRIVER = REPO_ROOT / "src" / "inputs" / "camera_driver"
CAMERA_TOOLS = Path(__file__).resolve().parent

for p in (REPO_ROOT, CAMERA_DRIVER, CAMERA_TOOLS):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
