"""
synthetic.py - render an ArUco marker at a KNOWN pose through a KNOWN lens.

Used by test_pose.py: the true distance is known exactly, so the pose code
can be checked without hardware or a tape measure. The lens model includes
distortion, so the real calibrations can be used.

    renderer = MarkerRenderer(camera_matrix, dist_coeffs, (1640, 1232))
    img, true_tvec = renderer.render(marker_id=10, size_m=0.12, distance_m=1.5,
                                     tilt_deg=(30, 0), offset=(0.3, -0.2))
"""

import cv2
import numpy as np

import _paths  # noqa: F401
from src.logic.vision import get_dictionary

TEX_PX_PER_MARKER = 600  # texture resolution of the marker (black square)


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class MarkerRenderer:
    def __init__(self, camera_matrix, dist_coeffs, image_size, dictionary="DICT_4X4_50"):
        self.K = np.asarray(camera_matrix, np.float64)
        self.dist = np.asarray(dist_coeffs, np.float64).reshape(1, -1)
        self.size = tuple(image_size)
        self.dictionary = get_dictionary(dictionary)
        w, h = self.size
        # For every output pixel: the undistorted, normalised camera ray.
        xs, ys = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        pts = np.stack([xs.ravel(), ys.ravel()], 1).reshape(-1, 1, 2)
        norm = cv2.undistortPointsIter(pts, self.K, self.dist, None, None,
                                       (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 40, 1e-7))
        self._rays = np.concatenate([norm.reshape(h, w, 2),
                                     np.ones((h, w, 1), np.float32)], axis=2)
        self._textures = {}

    def _texture(self, marker_id):
        """Marker with a white quiet zone of 1/4 of its size around it."""
        if marker_id not in self._textures:
            m = cv2.aruco.generateImageMarker(self.dictionary, marker_id, TEX_PX_PER_MARKER)
            pad = TEX_PX_PER_MARKER // 4
            self._textures[marker_id] = cv2.copyMakeBorder(m, pad, pad, pad, pad,
                                                           cv2.BORDER_CONSTANT, value=255)
        return self._textures[marker_id]

    def render(self, marker_id, size_m, distance_m, tilt_deg=(0.0, 0.0), offset=(0.0, 0.0),
               roll_deg=0.0, blur_sigma=0.7, noise_sigma=2.0, background=110, seed=0):
        """
        distance_m: distance along the line of sight to the marker centre.
        offset:     marker centre direction as a fraction of the half image size
                    (-1..1 in x and y; 0, 0 = optical axis).
        tilt_deg:   marker tilt about its own x and y axes (0, 0 = facing the camera).
        Returns (image_bgr, true_tvec, true_corners_px (4, 2)).
        """
        w, h = self.size
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        # Direction of the marker centre through an (undistorted) image point.
        u = cx + offset[0] * (w / 2)
        v = cy + offset[1] * (h / 2)
        ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        tvec = ray / np.linalg.norm(ray) * distance_m

        # Marker frame (x right, y up, z towards the viewer) -> camera frame.
        facing = np.diag([1.0, -1.0, -1.0])
        rz = cv2.Rodrigues(np.array([0, 0, np.radians(roll_deg)]))[0]
        R = facing @ _rot_x(np.radians(tilt_deg[0])) @ _rot_y(np.radians(tilt_deg[1])) @ rz

        # Plane homography: marker (X, Y, 1) -> normalised ray; invert it.
        Hn = np.column_stack([R[:, 0], R[:, 1], tvec])
        q = self._rays @ np.linalg.inv(Hn).T
        X, Y = q[..., 0] / q[..., 2], q[..., 1] / q[..., 2]

        # Anti-aliasing: shrink the texture (area-averaged) to ~1.5x the marker's
        # size in the image, so each output pixel averages the texture like a
        # real sensor pixel does. Point-sampling a much larger texture makes
        # grid-aligned edges step-shaped and biases the corners.
        base = self._texture(marker_id)
        est_px = fx * size_m / distance_m
        k = min(1.0, max(48.0, 1.5 * est_px) / TEX_PX_PER_MARKER)
        new_w = max(8, round(base.shape[1] * k))
        k = new_w / base.shape[1]
        tex = cv2.resize(base, (new_w, new_w), interpolation=cv2.INTER_AREA) if k < 1 else base
        pad = (TEX_PX_PER_MARKER // 4) * k
        scale = TEX_PX_PER_MARKER * k / size_m
        # Pixel-centre convention: the marker edge lies on a pixel BOUNDARY (-0.5).
        mapx = ((X + size_m / 2) * scale + pad - 0.5).astype(np.float32)
        mapy = ((size_m / 2 - Y) * scale + pad - 0.5).astype(np.float32)   # texture y goes down
        behind = q[..., 2] <= 0
        mapx[behind] = -1
        img = cv2.remap(tex, mapx, mapy, cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_CONSTANT, borderValue=background)

        rng = np.random.default_rng(seed)
        if blur_sigma > 0:
            img = cv2.GaussianBlur(img, (0, 0), blur_sigma)
        img = img.astype(np.float32) * 0.8 + 25           # not perfect black/white
        if noise_sigma > 0:
            img += rng.normal(0, noise_sigma, img.shape)
        img = np.clip(img, 0, 255).astype(np.uint8)

        h2 = size_m / 2
        obj = np.array([[-h2, h2, 0], [h2, h2, 0], [h2, -h2, 0], [-h2, -h2, 0]], np.float64)
        corners, _ = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], tvec, self.K, self.dist)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), tvec, corners.reshape(4, 2)

    def fully_visible(self, corners, margin_px=4):
        """True if the whole marker (plus a small margin) is inside the image."""
        w, h = self.size
        c = np.asarray(corners)
        return bool(np.all(c[:, 0] >= margin_px) and np.all(c[:, 0] < w - margin_px)
                    and np.all(c[:, 1] >= margin_px) and np.all(c[:, 1] < h - margin_px))
