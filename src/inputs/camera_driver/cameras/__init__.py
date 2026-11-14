"""Camera abstraction: one interface for USB and CSI cameras.

    from cameras import open_camera

    with open_camera() as cam:          # uses `active_camera` from config.yaml
        ok, frame, stamp = cam.read()
"""

from .base import Camera, CameraError
from .factory import get_camera, load_config, open_camera

__all__ = ["Camera", "CameraError", "get_camera", "load_config", "open_camera"]
