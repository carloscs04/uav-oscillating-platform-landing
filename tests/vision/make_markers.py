"""
make_markers.py - printable flight markers at exact size (one per A4 page).

    python3 tests/vision/make_markers.py                  # ids + size from src/logic/vision/config.yaml
    python3 tests/vision/make_markers.py --ids 10 --size-mm 60

Writes tests/vision/markers/markers_<dict>_<size>mm.pdf. Print at 100 % /
"actual size", then MEASURE the black square and put the real value in
src/logic/vision/config.yaml (aruco: marker_mm). Laser printers are preferred: the IMX219 is
NoIR and some inkjet blacks look grey in infrared.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

import _paths  # noqa: F401
from src.logic.vision.master_vision import load_vision_config
from src.logic.vision import get_dictionary

A4_MM = (210.0, 297.0)  # portrait
MM_PER_INCH = 25.4


def marker_page(dictionary_name, marker_id, size_mm, dpi):
    px_mm = dpi / MM_PER_INCH
    page = np.full((round(A4_MM[1] * px_mm), round(A4_MM[0] * px_mm)), 255, np.uint8)
    cells = 6  # 4x4 bits + 1-cell black border on each side
    cell_px = round(size_mm / cells * px_mm)
    side = cell_px * cells  # whole cells, so edges are crisp
    img = cv2.aruco.generateImageMarker(get_dictionary(dictionary_name), marker_id, side)
    y0 = (page.shape[0] - side) // 2 - round(15 * px_mm)
    x0 = (page.shape[1] - side) // 2
    page[y0:y0 + side, x0:x0 + side] = img

    # Cut guide one cell outside the marker: that white quiet zone must stay.
    m = cell_px
    cv2.rectangle(page, (x0 - m, y0 - m), (x0 + side + m, y0 + side + m), 160, 1)
    real_mm = side / px_mm
    label = [f"ID {marker_id}   {dictionary_name}",
             f"black square {real_mm:.1f} mm - print at 100 %, then measure it",
             "cut on the grey line (keep the white border)"]
    for k, text in enumerate(label):
        cv2.putText(page, text, (x0 - m, y0 + side + m + round((10 + 8 * k) * px_mm)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.13 * px_mm, 0, max(1, round(0.2 * px_mm)),
                    cv2.LINE_AA)  # ~3 mm text
    return page, real_mm


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config")
    ap.add_argument("--ids", type=int, nargs="+")
    ap.add_argument("--size-mm", type=float)
    ap.add_argument("--dpi", type=int, default=600)
    args = ap.parse_args()

    acfg = load_vision_config(args.config)["aruco"]
    ids = args.ids or acfg.get("ids") or [0]
    size = args.size_mm or float(acfg["marker_mm"])
    if size > A4_MM[0] - 30:
        raise SystemExit(f"{size} mm does not fit on A4 with its white border")

    from PIL import Image
    pages = []
    for marker_id in ids:
        page, real_mm = marker_page(acfg["dictionary"], marker_id, size, args.dpi)
        pages.append(Image.fromarray(page))
    out = Path(__file__).resolve().parent / "markers"
    out.mkdir(exist_ok=True)
    path = out / f"markers_{acfg['dictionary']}_{size:g}mm.pdf"
    pages[0].save(path, save_all=True, append_images=pages[1:], resolution=args.dpi)
    print(f"Wrote {path}: ids {ids}, black square {real_mm:.1f} mm, one per A4 page")


if __name__ == "__main__":
    main()
