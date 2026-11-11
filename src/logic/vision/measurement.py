"""Data types shared by the vision pipeline."""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class MarkerMeasurement:
    """Pose of one ArUco marker in one frame, in the CAMERA frame.

    Camera frame (OpenCV convention): x right, y down, z out of the lens.
    Units: metres, degrees, pixels.
    """

    marker_id: int
    stamp: float                 # capture time of the frame (seconds, time.monotonic())
    tvec: np.ndarray             # (3,) marker centre position [x, y, z] in metres
    rvec: np.ndarray             # (3,) marker rotation (Rodrigues vector)
    distance: float              # |tvec|, metres: straight-line camera -> marker centre
    reproj_err_px: float         # RMS pixel error of the fitted pose (quality)
    ambiguity: float             # 0..1, error ratio of the best vs. the flipped pose; near 1 = orientation ambiguous
    view_angle_deg: float        # 0 = marker faces the camera, 90 = seen edge-on
    marker_px: float             # mean side length of the marker in the image, pixels
    calibrated: bool             # False = nominal (spec-sheet) intrinsics, distances approximate
    camera_id: str = ""
    reliable: bool = True        # False if the pose failed quality checks
    corners: np.ndarray = field(default=None, repr=False)  # (4, 2) image corners, for drawing

    @property
    def x(self):
        return float(self.tvec[0])

    @property
    def y(self):
        return float(self.tvec[1])

    @property
    def z(self):
        return float(self.tvec[2])

    def to_dict(self):
        """JSON-friendly dict. Keeps the fields Bridge_WIFI's CameraTransmitter
        already uses (id, detected, x, y, z) and adds the rest."""
        return {
            "id": int(self.marker_id),
            "detected": True,
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "z": round(self.z, 4),
            "distance": round(float(self.distance), 4),
            "rvec": [round(float(v), 5) for v in self.rvec],
            "stamp": self.stamp,
            "reproj_err_px": round(float(self.reproj_err_px), 3),
            "ambiguity": round(float(self.ambiguity), 3),
            "view_angle_deg": round(float(self.view_angle_deg), 1),
            "marker_px": round(float(self.marker_px), 1),
            "calibrated": bool(self.calibrated),
            "reliable": bool(self.reliable),
            "camera": self.camera_id,
        }
