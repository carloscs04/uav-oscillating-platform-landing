"""Abstract camera interface shared by every backend (USB, CSI).

The rest of the vision system only uses this interface, so cameras can be
swapped by changing the config without touching calibration or pose code.
"""

import time
from abc import ABC, abstractmethod

import cv2


class CameraError(RuntimeError):
    """Raised when a camera cannot be found, opened, or configured."""


class Camera(ABC):
    backend = "base"

    def __init__(self, camera_id, resolution, fps=30.0, rotate_180=False,
                 warmup_frames=5):
        self.camera_id = camera_id
        self.resolution = (int(resolution[0]), int(resolution[1]))
        self.fps = float(fps)
        self.rotate_180 = bool(rotate_180)
        self.warmup_frames = int(warmup_frames)
        self._opened = False
        # Backends that can rotate in hardware set this to False.
        self._software_rotate = self.rotate_180

    # ------------------------------------------------------------------ API
    @property
    def calib_key(self):
        """Name used for calibration files, e.g. 'csi_imx219_1640x1232'."""
        w, h = self.resolution
        return f"{self.camera_id}_{w}x{h}"

    @property
    def is_open(self):
        return self._opened

    def open(self):
        """Open the device, let auto-exposure settle, and verify frame size."""
        if self._opened:
            return self
        self._open()
        self._opened = True
        try:
            frame = None
            for _ in range(max(1, self.warmup_frames)):
                frame = self._read()
            if frame is None:
                raise CameraError(f"{self.camera_id}: no frame received after opening")
            h, w = frame.shape[:2]
            if (w, h) != self.resolution:
                raise CameraError(
                    f"{self.camera_id}: requested {self.resolution[0]}x{self.resolution[1]} "
                    f"but camera delivers {w}x{h}")
        except Exception:
            self.close()
            raise
        return self

    def read(self):
        """Return (ok, frame_bgr, timestamp). Timestamp is time.monotonic() seconds."""
        if not self._opened:
            raise CameraError(f"{self.camera_id}: read() called before open()")
        frame = self._read()
        stamp = time.monotonic()
        if frame is None:
            return False, None, stamp
        if self._software_rotate:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        return True, frame, stamp

    def close(self):
        if self._opened:
            self._opened = False
            self._close()

    def info(self):
        """Description of the camera, useful for logs and calibration metadata."""
        return {
            "camera_id": self.camera_id,
            "backend": self.backend,
            "resolution": list(self.resolution),
            "fps": self.fps,
            "rotate_180": self.rotate_180,
        }

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    def __repr__(self):
        w, h = self.resolution
        return f"<{type(self).__name__} {self.camera_id} {w}x{h}@{self.fps:g}>"

    # ------------------------------------------------- backend implementation
    @abstractmethod
    def _open(self):
        """Open and configure the device."""

    @abstractmethod
    def _read(self):
        """Return one BGR frame (numpy array) or None."""

    @abstractmethod
    def _close(self):
        """Release the device."""
