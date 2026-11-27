"""Touchdown point from every visible pad marker, with no jump when the set of markers changes.

Each pad marker gives its own estimate of the touchdown point (pad_touchdown). They differ by a
centimetre or so (glueing / measuring), and when the camera went from ID 10 straight to ID 20 the
point jumped in one frame: ArduPilot's precision landing (moving-target estimator) read the step
as the pad moving fast and the drone slid away from the pad in the last 40 cm.

Here:
  * every visible pad marker is averaged, weighted by height: the big (far) marker above h_far,
    the small touchdown (near) marker below h_near, a linear hand-over in between;
  * when the set of markers in use changes (one appears or is lost), the output keeps going from
    where it was: the difference is carried as an offset that fades out with time constant tau_s
    (bumpless transfer). A 1 cm difference becomes a ~2 cm/s glide instead of a step."""
import math
from typing import Dict, Optional, Tuple

Point = Tuple[float, float]


class PadFusion:
    def __init__(self, near_id: int = 20, far_id: int = 10, h_near: float = 0.30, h_far: float = 0.45,
                 other_weight: float = 0.3, tau_s: float = 0.5, max_gap_s: float = 0.3):
        self.near_id, self.far_id = near_id, far_id
        self.h_near, self.h_far = h_near, h_far
        self.other_weight = other_weight
        self.tau_s = tau_s
        self.max_gap_s = max_gap_s
        self.reset()

    def reset(self):
        self._key = None
        self._off = (0.0, 0.0)
        self._last: Optional[Point] = None
        self._last_t: Optional[float] = None

    def weight(self, marker_id: int, h: Optional[float]) -> float:
        if marker_id not in (self.near_id, self.far_id):
            return self.other_weight
        if h is None:
            near = 0.0
        else:
            near = min(1.0, max(0.0, (self.h_far - h) / max(1e-6, self.h_far - self.h_near)))
        return near if marker_id == self.near_id else 1.0 - near

    def update(self, points: Dict[int, Point], h: Optional[float], now: float) -> Optional[Point]:
        """points: marker id -> (forward_m, right_m) touchdown estimate from that marker."""
        if not points:
            return None
        ws = {i: self.weight(i, h) for i in points}
        tot = sum(ws.values())
        if tot < 1e-6:                       # e.g. only the small marker, high up: use what there is
            ws, tot = {i: 1.0 for i in points}, float(len(points))
        raw = (sum(ws[i] * p[0] for i, p in points.items()) / tot,
               sum(ws[i] * p[1] for i, p in points.items()) / tot)
        key = frozenset(i for i, w in ws.items() if w > 0.0)
        recent = self._last_t is not None and 0.0 <= now - self._last_t <= self.max_gap_s
        if recent:
            k = math.exp(-(now - self._last_t) / self.tau_s)
            self._off = (self._off[0] * k, self._off[1] * k)
        else:
            self._off = (0.0, 0.0)
        if recent and key != self._key and self._last is not None:
            self._off = (self._last[0] - raw[0], self._last[1] - raw[1])
        self._key = key
        out = (raw[0] + self._off[0], raw[1] + self._off[1])
        self._last, self._last_t = out, now
        return out
