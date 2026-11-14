"""
ChArUco board definition, shared by capture and calibrate, plus a generator
for the printable board.

    python3 -m calibration.board            # writes calibration/files/charuco_<...>.pdf and .png
    python3 -m calibration.board --dpi 300

Print the PDF at 100% / "actual size" (NOT "fit to page"), glue it flat to
something rigid, then measure a square and update config.yaml.
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

if __package__ in (None, ""):  # allow `python3 calibration/board.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import _paths  # noqa: E402,F401  (camera layer lives in src/inputs/camera_driver)

from cameras import load_config  # noqa: E402

PAPER_MM = {"A4": (297.0, 210.0), "A3": (420.0, 297.0)}  # landscape
MM_PER_INCH = 25.4


def board_config(config=None):
    if not isinstance(config, dict):
        config = load_config(config)
    return config["charuco_board"]


def get_dictionary(name):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"unknown ArUco dictionary '{name}'")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def make_board(bcfg):
    """CharucoBoard with lengths in METRES, so poses come out in metres."""
    if bcfg["marker_mm"] >= bcfg["square_mm"]:
        raise ValueError("marker_mm must be smaller than square_mm")
    return cv2.aruco.CharucoBoard(
        (int(bcfg["squares_x"]), int(bcfg["squares_y"])),
        bcfg["square_mm"] / 1000.0,
        bcfg["marker_mm"] / 1000.0,
        get_dictionary(bcfg["dictionary"]),
    )


def make_detector(board):
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    charuco_params = cv2.aruco.CharucoParameters()
    charuco_params.tryRefineMarkers = True  # recover markers missed at first pass
    return cv2.aruco.CharucoDetector(board, charuco_params, params)


def detect_corners(detector, gray):
    """Return (charuco_corners Nx1x2, charuco_ids Nx1) or (None, None)."""
    corners, ids, _, _ = detector.detectBoard(gray)
    if ids is None or len(ids) == 0:
        return None, None
    return corners, ids


def n_corners(bcfg):
    return (int(bcfg["squares_x"]) - 1) * (int(bcfg["squares_y"]) - 1)


def describe(bcfg):
    return (f"{bcfg['squares_x']}x{bcfg['squares_y']} squares, "
            f"square {bcfg['square_mm']:g} mm, marker {bcfg['marker_mm']:g} mm, "
            f"{bcfg['dictionary']}")


def generate_printable(bcfg, out_dir, dpi=600):
    """Render the board centred on a landscape page at exact physical size."""
    paper = bcfg.get("paper", "A4")
    if paper not in PAPER_MM:
        raise ValueError(f"paper must be one of {list(PAPER_MM)}")
    page_mm = PAPER_MM[paper]
    px_per_mm = dpi / MM_PER_INCH
    sx, sy = int(bcfg["squares_x"]), int(bcfg["squares_y"])
    board_mm = (sx * bcfg["square_mm"], sy * bcfg["square_mm"])

    margin_mm = 8.0
    if board_mm[0] > page_mm[0] - 2 * margin_mm or board_mm[1] > page_mm[1] - 2 * margin_mm - 8:
        raise ValueError(
            f"board is {board_mm[0]:g}x{board_mm[1]:g} mm and does not fit on {paper}; "
            f"use smaller squares or a bigger paper")

    # Square size must be a whole number of pixels or squares come out uneven.
    square_px = round(bcfg["square_mm"] * px_per_mm)
    board_img = make_board(bcfg).generateImage((sx * square_px, sy * square_px),
                                               marginSize=0, borderBits=1)

    page_w, page_h = round(page_mm[0] * px_per_mm), round(page_mm[1] * px_per_mm)
    page = np.full((page_h, page_w), 255, dtype=np.uint8)
    y0 = (page_h - board_img.shape[0]) // 2 - round(4 * px_per_mm)  # leave room for the label
    x0 = (page_w - board_img.shape[1]) // 2
    page[y0:y0 + board_img.shape[0], x0:x0 + board_img.shape[1]] = board_img

    label = (f"ChArUco {describe(bcfg)} | {paper} landscape, print at 100% (actual size) | "
             f"measure a square and update config.yaml")
    scale = 0.35 * px_per_mm / 10
    cv2.putText(page, label, (x0, y0 + board_img.shape[0] + round(7 * px_per_mm)),
                cv2.FONT_HERSHEY_SIMPLEX, scale, 0, max(1, round(scale * 2)), cv2.LINE_AA)

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = (f"charuco_{sx}x{sy}_{bcfg['square_mm']:g}mm_{bcfg['marker_mm']:g}mm_"
            f"{bcfg['dictionary']}_{paper}")
    written = []
    try:
        from PIL import Image  # PDF/PNG with DPI metadata -> prints at true size

        img = Image.fromarray(page)
        for ext in ("pdf", "png"):
            path = out_dir / f"{stem}.{ext}"
            img.save(path, resolution=dpi) if ext == "pdf" else img.save(path, dpi=(dpi, dpi))
            written.append(path)
    except ImportError:
        path = out_dir / f"{stem}.png"
        cv2.imwrite(str(path), page)
        written.append(path)
        print("Pillow not installed: wrote PNG without DPI info. Print it scaled to "
              f"{page_mm[0]:g}x{page_mm[1]:g} mm.")
    return written, board_mm


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="path to config.yaml")
    ap.add_argument("--dpi", type=int, default=600)
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "files"),
                    help="output folder (default: calibration/files/)")
    args = ap.parse_args()

    bcfg = board_config(args.config)
    written, board_mm = generate_printable(bcfg, args.out, args.dpi)
    print(f"Board: {describe(bcfg)}, {n_corners(bcfg)} inner corners")
    print(f"Printed size should be {board_mm[0]:g} x {board_mm[1]:g} mm")
    for p in written:
        print(f"Wrote {p}")


if __name__ == "__main__":
    main()
