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


# Camera -> drone body, measured with the hand tests of 2026-11-22: moving the drone forward
# made x grow, moving it right made y grow, so the marker is at forward = -x_cam, right = -y_cam.
# Rows: body (forward, right) from camera (x, y). The camera's optical axis = body down.
CAM_TO_BODY = ((-1.0, 0.0), (0.0, -1.0))


def tilt_compensate(nx, ny, roll_rad, pitch_rad, lidar_range_m, cam_to_body=CAM_TO_BODY):
    """Marker offset from the drone centre in a LEVEL frame aligned with the drone's heading.

    A tilted drone sees a marker straight below it off-centre (~h*tan(tilt)); this removes it
    using the Pixhawk attitude (ATTITUDE roll/pitch, ArduPilot convention: roll right-wing-down
    positive, pitch nose-up positive). The lidar points along the body down axis, so its
    reading is a slant range: vertical height = range * cos(roll) * cos(pitch) (flat floor).

    Returns (forward_m, right_m, height_m)."""
    (a, b), (c, d) = cam_to_body
    v = np.array([a * nx + b * ny, c * nx + d * ny, 1.0])           # body (fwd, right, down)
    cr, sr, cp, sp = np.cos(roll_rad), np.sin(roll_rad), np.cos(pitch_rad), np.sin(pitch_rad)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    lv = ry @ rx @ v                                                  # level frame, yaw = heading
    h = float(lidar_range_m) * cr * cp
    if lv[2] <= 1e-6:
        return None, None, h
    return float(lv[0] / lv[2] * h), float(lv[1] / lv[2] * h), h


def _level_point(px, camera_matrix, dist_coeffs, roll_rad, pitch_rad, lidar_range_m):
    nx, ny = pixel_to_ray(px, camera_matrix, dist_coeffs)
    f, r, _ = tilt_compensate(nx, ny, roll_rad, pitch_rad, lidar_range_m)
    return None if f is None else np.array([f, r])


def pad_touchdown(corners, center_mm, camera_matrix, dist_coeffs, roll_rad, pitch_rad, lidar_range_m):
    """Touchdown point (pad origin) seen from ONE pad marker, in the level drone frame.

    The marker's position on the pad is known (config landing_pad.markers center_mm, pad x =
    right, y = FRONT, every marker glued with its printed top edge to the front). Its corners
    on the floor give the pad's right/front directions, so any pad marker points at the same
    spot: no jump when the target changes from ID 10 to ID 20. The marker size is not needed.

    Returns (forward_m, right_m) of the touchdown point, or None."""
    p = np.asarray(corners, np.float64).reshape(4, 2)  # OpenCV order: TL, TR, BR, BL (printed)
    g = [_level_point(c, camera_matrix, dist_coeffs, roll_rad, pitch_rad, lidar_range_m) for c in p]
    c = _level_point(marker_center_px(p), camera_matrix, dist_coeffs, roll_rad, pitch_rad, lidar_range_m)
    if c is None or any(x is None for x in g):
        return None
    tl, tr, br, bl = g
    ex = (tr - tl) + (br - bl)          # pad right
    ey = (tl - bl) + (tr - br)          # pad front
    nx_, ny_ = np.linalg.norm(ex), np.linalg.norm(ey)
    if nx_ < 1e-9 or ny_ < 1e-9:
        return None
    ex, ey = ex / nx_, ey / ny_
    t = c - (center_mm[0] / 1000.0) * ex - (center_mm[1] / 1000.0) * ey
    return float(t[0]), float(t[1])
