# file: src/logic/vision/height_check.py
"""Camera height (ArUco pose alone) vs lidar height: is the camera a good referee for the lidar?

The camera height is the distance from the lens to the marker PLANE (solvePnP pose): it does
not use the lidar and does not depend on the drone's tilt. The lidar height is the slant range
times cos(roll)*cos(pitch). Over a flat board both must agree; `summarize` says by how much,
per marker and per height band (the `camlidar` command, a hand test with the drone disarmed)."""

import cv2
import numpy as np

BINS_M = (0.10, 0.20, 0.30, 0.40, 0.60, 0.80, 1.00, 1.50, 2.50)
OK_BIAS_CM = 3.0      # mean (camera - lidar) within this -> usable as referee without correction
OK_SD_CM = 2.0        # and scatter within this
MIN_BIN_N = 10


def plane_height(rvec, tvec):
    """Distance (m) from the camera centre to the marker plane: |n . t|, n = marker Z axis."""
    R, _ = cv2.Rodrigues(np.asarray(rvec, np.float64).reshape(3))
    return float(abs(R[:, 2] @ np.asarray(tvec, np.float64).reshape(3)))


def _stats(d):
    m = sum(d) / len(d)
    return m, (sum((x - m) ** 2 for x in d) / len(d)) ** 0.5


def summarize(samples):
    """samples = [(marker_id, h_cam_m, h_lidar_m)] -> (text lines, {id: verdict dict})."""
    lines, verdicts = [], {}
    by_id = {}
    for mid, hc, hl in samples:
        if hc and hl and hc > 0 and hl > 0:
            by_id.setdefault(int(mid), []).append((float(hc), float(hl)))
    if not by_id:
        return ["camlidar: no samples with a marker AND the lidar at the same time."], {}
    for mid in sorted(by_id):
        v = by_id[mid]
        lines.append(f"#{mid}: {len(v)} samples, lidar {min(x[1] for x in v) * 100:.0f}-"
                     f"{max(x[1] for x in v) * 100:.0f} cm")
        lines.append("   height band     n   cam-lidar mean   sd    cam/lidar")
        bad = []
        for a, b in zip(BINS_M, BINS_M[1:]):
            s = [x for x in v if a <= x[1] < b]
            if len(s) < MIN_BIN_N:
                continue
            m, sd = _stats([(c - l) * 100 for c, l in s])
            ratio = sum(c / l for c, l in s) / len(s)
            ok = abs(m) <= OK_BIAS_CM and sd <= OK_SD_CM
            if not ok:
                bad.append(f"{a * 100:.0f}-{b * 100:.0f}")
            lines.append(f"   {a * 100:4.0f}-{b * 100:<4.0f} cm {len(s):5d}   {m:+7.1f} cm  {sd:5.1f} cm"
                         f"   {ratio:.3f}  {'OK' if ok else 'CHECK'}")
        n = len(v)
        sx = sum(l for _c, l in v); sy = sum(c for c, _l in v)
        sxx = sum(l * l for _c, l in v); sxy = sum(c * l for c, l in v)
        den = n * sxx - sx * sx
        k = (n * sxy - sx * sy) / den if den > 1e-9 else 1.0
        c0 = (sy - k * sx) / n
        m_all, sd_all = _stats([(c - l) * 100 for c, l in v])
        lines.append(f"   fit: cam = {k:.3f} x lidar {c0 * 100:+.1f} cm | overall {m_all:+.1f} cm, sd {sd_all:.1f} cm")
        ok = not bad and abs(m_all) <= OK_BIAS_CM and sd_all <= OK_SD_CM
        lines.append(f"   -> #{mid} {'OK as lidar referee' if ok else 'NEEDS CORRECTION in ' + (', '.join(bad) or 'overall')}")
        verdicts[mid] = {"ok": ok, "k": k, "c_m": c0, "bias_cm": m_all, "sd_cm": sd_all, "n": n}
    return lines, verdicts
