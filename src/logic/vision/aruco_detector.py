"""ArUco marker detection: image -> marker ids + sub-pixel corners.

Pure function of the image: no camera, threads or network, so it can be
tested on files.
"""

import cv2
import numpy as np


def get_dictionary(name):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"unknown ArUco dictionary '{name}'")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def refine_corners_edges(gray, corners, samples=24, min_contrast=20.0):
    """
    Re-locate the 4 marker corners by fitting a straight line to each edge
    of the black square and intersecting neighbouring lines.

    Why: OpenCV's SUBPIX refinement is designed for chessboard (saddle)
    corners. On a marker's outer corners it is pulled slightly INTO the black
    square, the marker looks smaller and the distance comes out too long
    (measured +19 mm at 3 m). Edge-line fitting is unbiased under symmetric
    blur, like CORNER_REFINE_APRILTAG, but ~100x faster because it only
    looks at the 4 edges of already-detected markers.

    Each edge point is where the intensity profile across the edge crosses
    halfway between the black border and the white margin.
    """
    pts = np.asarray(corners, np.float64).reshape(4, 2)
    side = float(np.mean(np.linalg.norm(pts - np.roll(pts, 1, axis=0), axis=1)))
    if side < 12:
        return corners  # too small to refine reliably
    # Stay inside the black border (side/6 wide for a 4x4 marker) on the inner side.
    search = max(1.5, side / 12.0)
    steps = np.linspace(-search, search, int(search * 8) + 1)   # ~1/4 px spacing
    img = gray.astype(np.float32)

    lines = []
    for i in range(4):
        a, b = pts[i], pts[(i + 1) % 4]
        d = (b - a) / np.linalg.norm(b - a)
        outward = np.array([d[1], -d[0]])          # corners run clockwise in the image
        t = np.linspace(0.15, 0.85, samples)        # keep away from the corners
        base = a + (b - a) * t[:, None]
        grid = base[:, None, :] + steps[None, :, None] * outward
        prof = cv2.remap(img, grid[..., 0].astype(np.float32), grid[..., 1].astype(np.float32),
                         cv2.INTER_LINEAR)
        k = max(2, len(steps) // 6)
        edge_pts = []
        for row, p0 in zip(prof, base):
            lo, hi = row[:k].mean(), row[-k:].mean()
            if hi - lo < min_contrast:
                continue
            above = np.nonzero(row >= (lo + hi) / 2)[0]
            if len(above) == 0 or above[0] == 0:
                continue
            j = above[0]
            frac = ((lo + hi) / 2 - row[j - 1]) / (row[j] - row[j - 1])
            s = steps[j - 1] + frac * (steps[j] - steps[j - 1])
            edge_pts.append(p0 + s * outward)
        if len(edge_pts) < samples // 2:
            return corners  # edge not clean enough (occlusion, glare): keep the original
        vx, vy, x0, y0 = cv2.fitLine(np.array(edge_pts, np.float32), cv2.DIST_HUBER,
                                     0, 0.01, 0.01).ravel()
        lines.append((np.array([x0, y0]), np.array([vx, vy])))

    refined = np.empty((4, 2))
    for i in range(4):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]   # corner i joins edge i-1 and edge i
        A = np.column_stack([d1, -d2])
        if abs(np.linalg.det(A)) < 1e-6:
            return corners
        s = np.linalg.solve(A, p2 - p1)[0]
        refined[i] = p1 + s * d1
    # Sanity check: refinement only nudges corners, it never moves them far.
    if np.max(np.linalg.norm(refined - pts, axis=1)) > max(3.0, side * 0.05):
        return corners
    return refined.astype(np.float32)


class ArucoDetector:
    """
    detector = ArucoDetector("DICT_4X4_50", ids=[10, 25, 32])
    for marker_id, corners in detector.detect(gray):   # corners: (4, 2) float32
        ...

    Corner order (OpenCV): top-left, top-right, bottom-right, bottom-left of
    the marker as printed, whatever its rotation in the image.
    """

    REFINE_METHODS = ("edges", "subpix", "none")

    def __init__(self, dictionary="DICT_4X4_50", ids=None, refine="edges"):
        """refine: 'edges' (default, unbiased: see refine_corners_edges),
        'subpix' (OpenCV's, biased outward), or 'none'."""
        if refine not in self.REFINE_METHODS:
            raise ValueError(f"refine must be one of {self.REFINE_METHODS}")
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = (cv2.aruco.CORNER_REFINE_SUBPIX if refine == "subpix"
                                         else cv2.aruco.CORNER_REFINE_NONE)
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 50
        params.cornerRefinementMinAccuracy = 0.01
        self.dictionary_name = dictionary
        self.refine = refine
        self.ids = set(int(i) for i in ids) if ids else None
        self._detector = cv2.aruco.ArucoDetector(get_dictionary(dictionary), params)

    def detect(self, image):
        """Return [(marker_id, corners (4, 2) float32), ...] sorted by id."""
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            return []
        found = []
        for marker_id, c in zip(ids.ravel().tolist(), corners):
            if self.ids is None or marker_id in self.ids:
                c = c.reshape(4, 2).astype(np.float32)
                if self.refine == "edges":
                    c = refine_corners_edges(gray, c)
                found.append((marker_id, c))
        return sorted(found, key=lambda m: m[0])
