"""Move flat flight logs into one folder per flight (safe to run again).

experiments/flights/<date>/flight_<HH-MM-SS>_<codename>.csv / _events.log / _plnd*.*
  -> experiments/flights/<date>/<HH-MM-SS>_<codename>/<same file name>

Run: python3 tools/organize_flights.py [flights_dir]   (default: <repo>/experiments/flights)"""
import os
import re
import sys

PATTERN = re.compile(r"^flight_(\d{2}-\d{2}-\d{2})_([a-z]+)(\.csv|_.+)$")


def organize(root: str) -> int:
    moved = 0
    for day in sorted(os.listdir(root)):
        day_dir = os.path.join(root, day)
        if not os.path.isdir(day_dir):
            continue
        for name in sorted(os.listdir(day_dir)):
            src = os.path.join(day_dir, name)
            m = PATTERN.match(name)
            if not m or not os.path.isfile(src):
                continue
            dest_dir = os.path.join(day_dir, f"{m.group(1)}_{m.group(2)}")
            os.makedirs(dest_dir, exist_ok=True)
            dest = os.path.join(dest_dir, name)
            if os.path.exists(dest):
                print(f"skip (exists): {dest}")
                continue
            os.rename(src, dest)
            moved += 1
    return moved


if __name__ == "__main__":
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(repo, "experiments", "flights")
    print(f"moved {organize(root)} files under {root}")
