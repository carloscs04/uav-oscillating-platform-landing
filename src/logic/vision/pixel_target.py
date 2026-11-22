"""Target position from pixels + lidar height (any marker, any size).

The marker's size is not needed: only the image position of its centre.

    centre (pixels) -> undistorted ray (nx, ny) = tan(angle off the optical axis)
    lateral offset  =  (nx, ny) * height        (height from the lidar)

Camera frame (OpenCV): x = right in the image, y = DOWN in the image. (0, 0) means the
marker's centre sits on the optical axis, i.e. straight below the lens (= drone centre:
the camera is mounted at the centre).
"""

import cv2
import numpy as np


def marker_center_px(corners):
    """Centre of a marker in the image = intersection of its two diagonals.

    Unlike the mean of the 4 corners, this stays exact under perspective (a tilted
    marker's near half looks bigger, which pulls the corner mean towards it)."""
    p = np.asarray(corners, np.float64).reshape(4, 2)
    d1, d2 = p[2] - p[0], p[3] - p[1]
    A = np.column_stack([d1, -d2])
    if abs(np.linalg.det(A)) < 1e-9:
        return p.mean(axis=0)
    s = np.linalg.solve(A, p[1] - p[0])[0]
    return p[0] + s * d1


def pixel_to_ray(center_px, camera_matrix, dist_coeffs):
    """(u, v) pixels -> (nx, ny): undistorted normalised image coordinates."""
    pt = np.asarray(center_px, np.float64).reshape(1, 1, 2)
    n = cv2.undistortPoints(pt, np.asarray(camera_matrix, np.float64),
                            np.asarray(dist_coeffs, np.float64))
    return float(n[0, 0, 0]), float(n[0, 0, 1])


def pixel_target(corners, camera_matrix, dist_coeffs, height_m=None):
    """Everything the controller needs from one marker. height_m = lidar distance to the
    surface the marker lies on; None -> no metric offset (pixels and angles only)."""
    K = np.asarray(camera_matrix, np.float64)
    u, v = marker_center_px(corners)
    nx, ny = pixel_to_ray((u, v), K, dist_coeffs)
    out = {
        "u_px": round(float(u), 2), "v_px": round(float(v), 2),
        "du_px": round(float(u - K[0, 2]), 2), "dv_px": round(float(v - K[1, 2]), 2),
        "nx": round(nx, 5), "ny": round(ny, 5),
        "angle_x_deg": round(float(np.degrees(np.arctan(nx))), 2),
        "angle_y_deg": round(float(np.degrees(np.arctan(ny))), 2),
        "h_m": None, "x_m": None, "y_m": None, "err_m": None,
    }
    if height_m is not None and height_m > 0:
        x, y = nx * height_m, ny * height_m
        out.update({"h_m": round(float(height_m), 3), "x_m": round(x, 4), "y_m": round(y, 4),
                    "err_m": round(float(np.hypot(x, y)), 4)})
    return out
