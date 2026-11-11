"""Make `src.*` (repo root) and the camera tools (tests/camera) importable
from any tests/vision script, wherever it is run from."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CAMERA_TOOLS = REPO_ROOT / "tests" / "camera"

for p in (REPO_ROOT, CAMERA_TOOLS):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
