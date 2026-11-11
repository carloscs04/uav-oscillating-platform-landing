"""Marker pose from its 4 image corners: position, distance and quality.

Uses solvePnP with IPPE_SQUARE, the solver designed for square planar
markers. It returns both possible poses (a square seen nearly face-on can
be explained by two mirror-image tilts); the ratio of their errors tells
how ambiguous the orientation is. Distance is barely affected by that
ambiguity, orientation is.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from .measurement import MarkerMeasurement


@dataclass
class CameraModel:
    """Intrinsics used for pose estimation."""

    camera_matrix: np.ndarray    # 3x3
    dist_coeffs: np.ndarray      # 1xN
    image_size: tuple            # (width, height)
    calibrated: bool
    camera_id: str = ""

    @classmethod
    def from_calibration(cls, calib, camera_id=""):
        """From calibration.params.Calibration (load_calibration())."""
        return cls(np.asarray(calib.camera_matrix, np.float64),
                   np.asarray(calib.dist_coeffs, np.float64).reshape(1, -1),
                   tuple(calib.image_size), True, camera_id)

    @classmethod
    def nominal(cls, image_size, focal_px, camera_id=""):
        """Approximate model (focal length from the datasheet, no distortion).
        Distances are only as good as focal_px: use until calibrated."""
        w, h = image_size
        K = np.array([[focal_px, 0, w / 2], [0, focal_px, h / 2], [0, 0, 1]], np.float64)
        return cls(K, np.zeros((1, 5)), (w, h), False, camera_id)


def marker_object_points(size_m):
    """Marker corners in the marker frame, in IPPE_SQUARE order
    (top-left, top-right, bottom-right, bottom-left; x right, y up, z out of the marker)."""
    h = size_m / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)


def estimate_pose(marker_id, corners, size_m, camera, stamp=0.0, max_reproj_err_px=2.0):
    """
    corners: (4, 2) image points from ArucoDetector.
    size_m:  real side length of the black square, metres.
    Returns a MarkerMeasurement, or None if no pose could be computed.
    """
    obj = marker_object_points(size_m)
    img = np.asarray(corners, np.float64).reshape(4, 1, 2)
    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
        obj, img, camera.camera_matrix, camera.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not n:
        return None

    errs = np.asarray(errs, float).ravel()
    order = np.argsort(errs)
    best = int(order[0])
    rvec, tvec = rvecs[best].ravel(), tvecs[best].ravel()
    err = float(errs[best])
    ambiguity = float(errs[best] / errs[order[1]]) if n > 1 and errs[order[1]] > 0 else 0.0

    # Angle between the marker's normal and the line of sight.
    R, _ = cv2.Rodrigues(rvec)
    los = tvec / np.linalg.norm(tvec)
    view_angle = float(np.degrees(np.arccos(np.clip(abs(R[:, 2] @ -los), 0.0, 1.0))))

    pts = np.asarray(corners, float).reshape(4, 2)
    marker_px = float(np.mean(np.linalg.norm(pts - np.roll(pts, 1, axis=0), axis=1)))

    reliable = bool(tvec[2] > 0 and err <= max_reproj_err_px)
    return MarkerMeasurement(
        marker_id=int(marker_id), stamp=stamp, tvec=tvec, rvec=rvec,
        distance=float(np.linalg.norm(tvec)), reproj_err_px=err, ambiguity=ambiguity,
        view_angle_deg=view_angle, marker_px=marker_px, calibrated=camera.calibrated,
        camera_id=camera.camera_id, reliable=reliable, corners=pts.astype(np.float32))


def estimate_all(detections, sizes_m, camera, stamp=0.0, max_reproj_err_px=2.0):
    """detections from ArucoDetector.detect(); sizes_m: {id: size} or a single float."""
    out = []
    for marker_id, corners in detections:
        size = sizes_m.get(marker_id) if isinstance(sizes_m, dict) else sizes_m
        if not size:
            continue
        m = estimate_pose(marker_id, corners, size, camera, stamp, max_reproj_err_px)
        if m is not None:
            out.append(m)
    return out
