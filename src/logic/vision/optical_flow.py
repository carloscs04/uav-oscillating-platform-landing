# File: src/logic/vision/optical_flow.py

from typing import Optional, Tuple
import cv2
import numpy as np


class OpticalFlowEstimator:
    """Robust Lucas-Kanade sparse optical flow estimator for indoor EKF3 velocity fusion.

    Includes CLAHE low-light contrast enhancement, photometric error filtering, spatial MAD
    outlier rejection, motion-blur gating, and slew-rate clamping.
    """

    def __init__(
        self,
        native_resolution: Tuple[int, int] = (640, 360),
        focal_px: Tuple[float, float] = (654.0, 654.0),
        work_width: int = 320,
        max_corners: int = 75,
        min_corners: int = 20,
        swap_xy: bool = False,
        invert_x: bool = True,
        invert_y: bool = False,
        min_quality: int = 30,
        max_flow_rad_s: float = 4.5,
        smoothing: float = 0.85,
        focal_scale: float = 1.0,
    ):
        self.native_w = max(1, int(native_resolution[0]))
        self.native_h = max(1, int(native_resolution[1]))
        self.work_w = max(96, int(work_width))
        self.scale = self.work_w / float(self.native_w)
        self.work_h = max(54, int(round(self.native_h * self.scale)))

        f_scale = max(0.3, min(3.0, float(focal_scale)))
        self.fx_scaled = max(1.0, (float(focal_px[0]) / f_scale) * self.scale)
        self.fy_scaled = max(1.0, (float(focal_px[1]) / f_scale) * self.scale)

        self.max_corners = int(max_corners)
        self.min_corners = int(min_corners)
        self.swap_xy = bool(swap_xy)
        self.invert_x = bool(invert_x)
        self.invert_y = bool(invert_y)
        self.min_quality = int(min_quality)
        self.max_flow_rad_s = float(max_flow_rad_s)
        self.smoothing = max(0.1, min(1.0, float(smoothing)))

        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts: Optional[np.ndarray] = None
        self._prev_stamp: Optional[float] = None
        self._filt_rate_x: float = 0.0
        self._filt_rate_y: float = 0.0

        self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))

        self._lk_params = dict(
            winSize=(17, 17),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 25, 0.02),
        )
        self._feature_params = dict(
            maxCorners=self.max_corners,
            qualityLevel=0.015,
            minDistance=8,
            blockSize=7,
        )

    def reset(self):
        """Clears stored reference frame, tracked feature points, and filter state."""
        self._prev_gray = None
        self._prev_pts = None
        self._prev_stamp = None
        self._filt_rate_x = 0.0
        self._filt_rate_y = 0.0

    def _reject_frame(self, small: np.ndarray, stamp: float, dt_s: float) -> dict:
        self._prev_gray = small
        self._prev_pts = cv2.goodFeaturesToTrack(small, mask=None, **self._feature_params)
        self._prev_stamp = stamp
        self._filt_rate_x *= 0.4
        self._filt_rate_y *= 0.4
        return {
            "rad_x": 0.0,
            "rad_y": 0.0,
            "dt_s": max(1e-3, min(0.5, dt_s)),
            "quality": 0,
            "px_dx": 0.0,
            "px_dy": 0.0,
        }

    def update(self, gray_frame: np.ndarray, stamp: float) -> Optional[dict]:
        """Processes a new grayscale frame and returns filtered angular optical flow metrics."""
        if gray_frame is None or gray_frame.size == 0:
            return None

        resized = cv2.resize(gray_frame, (self.work_w, self.work_h), interpolation=cv2.INTER_AREA)
        small = self._clahe.apply(resized)

        if self._prev_gray is None or self._prev_stamp is None:
            self._prev_gray = small
            self._prev_pts = cv2.goodFeaturesToTrack(small, mask=None, **self._feature_params)
            self._prev_stamp = stamp
            return None

        dt_s = float(stamp - self._prev_stamp)
        if dt_s <= 1e-4 or dt_s > 0.25:
            return self._reject_frame(small, stamp, dt_s)

        # Sharpness / motion-blur check (Laplacian variance)
        sharpness = float(cv2.Laplacian(small, cv2.CV_64F).var())
        if sharpness < 8.0:
            return self._reject_frame(small, stamp, dt_s)

        if self._prev_pts is None or len(self._prev_pts) < self.min_corners:
            self._prev_pts = cv2.goodFeaturesToTrack(self._prev_gray, mask=None, **self._feature_params)

        if self._prev_pts is None or len(self._prev_pts) < 4:
            return self._reject_frame(small, stamp, dt_s)

        next_pts, status, err = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, small, self._prev_pts, None, **self._lk_params
        )

        if status is None or err is None:
            return self._reject_frame(small, stamp, dt_s)

        # Keep only points with valid status AND low photometric tracking error
        valid_mask = (status.flatten() == 1) & (err.flatten() <= 12.0)
        if np.count_nonzero(valid_mask) < 5:
            return self._reject_frame(small, stamp, dt_s)

        good_old = self._prev_pts[valid_mask].reshape(-1, 2)
        good_new = next_pts[valid_mask].reshape(-1, 2)
        deltas = good_new - good_old

        # Spatial consistency check (MAD): discard points that jumped inconsistently due to vibration
        med_dx = float(np.median(deltas[:, 0]))
        med_dy = float(np.median(deltas[:, 1]))
        dist_from_med = np.hypot(deltas[:, 0] - med_dx, deltas[:, 1] - med_dy)
        mad = float(np.median(dist_from_med))
        inlier_thresh = max(1.2, 2.8 * mad)
        inliers = dist_from_med <= inlier_thresh

        inlier_count = int(np.count_nonzero(inliers))
        if inlier_count < 5:
            return self._reject_frame(small, stamp, dt_s)

        # Average over clean inliers gives sub-pixel precision superior to raw median
        dx_px = float(np.mean(deltas[inliers, 0]))
        dy_px = float(np.mean(deltas[inliers, 1]))

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

        # Compute angular rates (rad/s) and clamp physical outliers
        rate_x = rad_x / dt_s
        rate_y = rad_y / dt_s
        if abs(rate_x) > self.max_flow_rad_s or abs(rate_y) > self.max_flow_rad_s:
            return self._reject_frame(small, stamp, dt_s)

        # Low-lag low-pass filter on angular rates to damp jitter without inducing phase delay
        a = self.smoothing
        self._filt_rate_x = a * rate_x + (1.0 - a) * self._filt_rate_x
        self._filt_rate_y = a * rate_y + (1.0 - a) * self._filt_rate_y

        rad_x_out = self._filt_rate_x * dt_s
        rad_y_out = self._filt_rate_y * dt_s

        # Quality combines inlier count ratio and spatial consensus (tightness of inliers)
        count_score = min(1.0, inlier_count / max(1.0, self.max_corners * 0.5))
        consensus_score = max(0.0, min(1.0, 1.0 - (mad / 3.5)))
        quality = int(round(255.0 * count_score * (0.6 + 0.4 * consensus_score)))

        if quality < self.min_quality:
            rad_x_out = 0.0
            rad_y_out = 0.0
            dx_px = 0.0
            dy_px = 0.0
            quality = 0

        # Refresh feature set for next step
        self._prev_gray = small
        self._prev_stamp = stamp
        if inlier_count < self.min_corners:
            self._prev_pts = cv2.goodFeaturesToTrack(small, mask=None, **self._feature_params)
        else:
            self._prev_pts = good_new[inliers].reshape(-1, 1, 2)

        return {
            "rad_x": rad_x_out,
            "rad_y": rad_y_out,
            "dt_s": dt_s,
            "quality": quality,
            "px_dx": dx_px,
            "px_dy": dy_px,
        }