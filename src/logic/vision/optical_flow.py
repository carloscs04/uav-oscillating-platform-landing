# File: src/logic/vision/optical_flow.py

from typing import Optional, Tuple
import cv2
import numpy as np


class OpticalFlowEstimator:
    """Lightweight Lucas-Kanade sparse optical flow estimator for indoor EKF3 velocity fusion.

    Downsamples incoming grayscale frames to a compact working resolution so frame-to-frame
    angular displacement (radians) is computed in ~1-2 ms without adding latency to ArUco tracking.
    """

    def __init__(
        self,
        native_resolution: Tuple[int, int] = (640, 480),
        focal_px: Tuple[float, float] = (500.0, 500.0),
        work_width: int = 160,
        max_corners: int = 40,
        min_corners: int = 12,
        swap_xy: bool = True,
        invert_x: bool = True,
        invert_y: bool = False,
    ):
        self.native_w = max(1, int(native_resolution[0]))
        self.native_h = max(1, int(native_resolution[1]))
        self.work_w = max(64, int(work_width))
        self.scale = self.work_w / float(self.native_w)
        self.work_h = max(48, int(round(self.native_h * self.scale)))

        self.fx_scaled = max(1.0, float(focal_px[0]) * self.scale)
        self.fy_scaled = max(1.0, float(focal_px[1]) * self.scale)

        self.max_corners = int(max_corners)
        self.min_corners = int(min_corners)
        self.swap_xy = bool(swap_xy)
        self.invert_x = bool(invert_x)
        self.invert_y = bool(invert_y)

        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts: Optional[np.ndarray] = None
        self._prev_stamp: Optional[float] = None

        self._lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 15, 0.03),
        )
        self._feature_params = dict(
            maxCorners=self.max_corners,
            qualityLevel=0.015,
            minDistance=8,
            blockSize=5,
        )

    def reset(self):
        """Clears stored reference frame and tracked feature points."""
        self._prev_gray = None
        self._prev_pts = None
        self._prev_stamp = None

    def update(self, gray_frame: np.ndarray, stamp: float) -> Optional[dict]:
        """Processes a new grayscale frame and returns angular optical flow metrics.

        Returns None on the very first frame or when timestamp delta is invalid; otherwise returns:
            {
                "rad_x": float,       # Integrated angular flow around sensor X axis (radians)
                "rad_y": float,       # Integrated angular flow around sensor Y axis (radians)
                "dt_s": float,        # Integration interval between frames (seconds)
                "quality": int,       # Tracking confidence 0..255 (0 = unusable)
                "px_dx": float,       # Median pixel shift X in working frame
                "px_dy": float,       # Median pixel shift Y in working frame
            }
        """
        if gray_frame is None or gray_frame.size == 0:
            return None

        small = cv2.resize(gray_frame, (self.work_w, self.work_h), interpolation=cv2.INTER_AREA)

        if self._prev_gray is None or self._prev_stamp is None:
            self._prev_gray = small
            self._prev_pts = cv2.goodFeaturesToTrack(small, mask=None, **self._feature_params)
            self._prev_stamp = stamp
            return None

        dt_s = float(stamp - self._prev_stamp)
        if dt_s <= 1e-4 or dt_s > 0.5:
            self._prev_gray = small
            self._prev_pts = cv2.goodFeaturesToTrack(small, mask=None, **self._feature_params)
            self._prev_stamp = stamp
            return {
                "rad_x": 0.0,
                "rad_y": 0.0,
                "dt_s": max(1e-3, min(0.5, dt_s)),
                "quality": 0,
                "px_dx": 0.0,
                "px_dy": 0.0,
            }

        if self._prev_pts is None or len(self._prev_pts) < self.min_corners:
            self._prev_pts = cv2.goodFeaturesToTrack(self._prev_gray, mask=None, **self._feature_params)

        if self._prev_pts is None or len(self._prev_pts) == 0:
            self._prev_gray = small
            self._prev_stamp = stamp
            return {
                "rad_x": 0.0,
                "rad_y": 0.0,
                "dt_s": dt_s,
                "quality": 0,
                "px_dx": 0.0,
                "px_dy": 0.0,
            }

        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, small, self._prev_pts, None, **self._lk_params
        )

        valid_mask = (status.flatten() == 1) if status is not None else np.array([], dtype=bool)
        good_old = self._prev_pts[valid_mask].reshape(-1, 2) if np.any(valid_mask) else np.empty((0, 2))
        good_new = next_pts[valid_mask].reshape(-1, 2) if np.any(valid_mask) else np.empty((0, 2))

        valid_count = len(good_new)
        if valid_count < 3:
            self._prev_gray = small
            self._prev_pts = cv2.goodFeaturesToTrack(small, mask=None, **self._feature_params)
            self._prev_stamp = stamp
            return {
                "rad_x": 0.0,
                "rad_y": 0.0,
                "dt_s": dt_s,
                "quality": 0,
                "px_dx": 0.0,
                "px_dy": 0.0,
            }

        # Median displacement rejects moving outliers and specular glare spikes
        deltas = good_new - good_old
        dx_px = float(np.median(deltas[:, 0]))
        dy_px = float(np.median(deltas[:, 1]))

        rad_x = dx_px / self.fx_scaled
        rad_y = dy_px / self.fy_scaled

        if self.swap_xy:
            rad_x, rad_y = rad_y, rad_x
            dx_px, dy_px = dy_px, dx_px
        if self.invert_x:
            rad_x = -rad_x
            dx_px = -dx_px
        if self.invert_y:
            rad_y = -rad_y
            dy_px = -dy_px

        quality = min(255, int(round((valid_count / max(1.0, self.max_corners * 0.75)) * 255.0)))

        # Refresh feature set for next step
        self._prev_gray = small
        self._prev_stamp = stamp
        if valid_count < self.min_corners:
            self._prev_pts = cv2.goodFeaturesToTrack(small, mask=None, **self._feature_params)
        else:
            self._prev_pts = good_new.reshape(-1, 1, 2)

        return {
            "rad_x": rad_x,
            "rad_y": rad_y,
            "dt_s": dt_s,
            "quality": quality,
            "px_dx": dx_px,
            "px_dy": dy_px,
        }