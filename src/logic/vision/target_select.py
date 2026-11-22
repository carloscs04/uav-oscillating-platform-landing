"""Which marker is the target: registered (known) markers by priority, else the nearest unknown.

Labels are "<FAMILY> #<ID>" (e.g. "APRILTAG_36h11 #3", "4X4_50 #10"), the text the panel shows.

    any known marker visible  -> the known one highest in the registry list
    only unknown markers      -> the one nearest the optical axis; kept while it stays in view
                                 (no jumping between unknowns from frame to frame)
    nothing                   -> no target
"""

from pathlib import Path

import numpy as np
import yaml

DEFAULT_REGISTRY = Path(__file__).resolve().parents[2] / "inputs" / "camera_driver" / "known_markers.yaml"


def normalize_label(text):
    """'DICT_APRILTAG_36h11#3', 'apriltag_36h11 # 3' -> 'APRILTAG_36h11 #3' (family case kept
    as OpenCV writes it, so it matches the scan labels)."""
    family, sep, marker_id = str(text).partition("#")
    if not sep:
        raise ValueError(f"bad marker label {text!r}: expected '<FAMILY> #<ID>'")
    family = family.strip()
    if family.upper().startswith("DICT_"):
        family = family[5:]
    # OpenCV names: upper case except the AprilTag 'h' (36h11, 25h9, 16h5)
    family = family.upper().replace("H11", "h11").replace("H9", "h9").replace("H5", "h5")
    return f"{family} #{int(marker_id.strip())}"


def load_registry(path=None):
    """Ordered list of known labels (index 0 = highest priority). Missing file -> []."""
    p = Path(path or DEFAULT_REGISTRY)
    if not p.exists():
        return []
    with open(p) as f:
        data = yaml.safe_load(f) or {}
    out = []
    for item in data.get("known") or []:
        label = normalize_label(item)
        if label not in out:
            out.append(label)
    return out


def marker_centre(corners):
    return np.asarray(corners, np.float64).reshape(4, 2).mean(axis=0)


def select_target(candidates, registry, ref_px, previous_label=None):
    """candidates: [(label, corners), ...]; registry: ordered known labels; ref_px: the fixed
    cross (principal point). Returns (index or None, known: bool)."""
    if not candidates:
        return None, False
    rank = {label: i for i, label in enumerate(registry)}
    known = [(rank[c[0]], i) for i, c in enumerate(candidates) if c[0] in rank]
    if known:
        return min(known)[1], True
    for i, c in enumerate(candidates):
        if c[0] == previous_label:
            return i, False
    ref = np.asarray(ref_px, np.float64)
    dist = [float(np.linalg.norm(marker_centre(c[1]) - ref)) for c in candidates]
    return int(np.argmin(dist)), False
