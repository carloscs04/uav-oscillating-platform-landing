"""
make_pad.py - printable landing mat on A4 tiles, from config.yaml `landing_pad`.

    python3 tests/vision/make_pad.py                 # onboard config (src/inputs/camera_driver)
    python3 tests/vision/make_pad.py --margin-mm 6   # printer's unprintable border (default 8)

Writes to tests/vision/markers/:
  pad_tiles.pdf    speckle background for the whole mat, cut into A4 tiles. Each tile has
                   crop marks: trim on them and butt the tiles edge to edge. White boxes
                   show exactly where each marker is glued.
  pad_markers.pdf  the markers with their white margin. Cut on the grey line and glue each
                   one into its box with the "FRONT" side towards the mat's FRONT.
  pad_preview.png  the assembled mat, to check the layout before printing.

Then it runs a self-test: simulated top-down camera views of the assembled mat at several
heights, reporting which markers are detected and how many flow features the speckle gives.

Why speckle: the optical flow uses the SAME camera. Over a plain mat the flow has nothing to
track exactly when hovering above the pad. The random shapes (3 size scales) give features
from ~0.15 m to ~2 m.

Print BOTH files at 100 % / "Actual size" (no "fit to page"), check the 100 mm scale bar
with a ruler, then measure the glued markers and put the measured values in config.yaml.
"""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

import _paths  # noqa: F401
from src.logic.vision import ArucoDetector, get_dictionary

DEFAULT_CONFIG = _paths.REPO_ROOT / "src" / "inputs" / "camera_driver" / "config.yaml"
OUT_DIR = Path(__file__).resolve().parent / "markers"
A4_MM = (210.0, 297.0)          # portrait (width, height)
MM_PER_INCH = 25.4

# Camera used by the self-test (usb_0bdc_8088 640x360 calibration) and the flow settings.
CAM_F_PX, CAM_W, CAM_H = 654.0, 640, 360
FLOW_WORK_W = 160


# --------------------------------------------------------------------------- layout
def load_pad(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    pad = cfg.get("landing_pad")
    if not pad:
        raise SystemExit(f"no 'landing_pad' section in {path}")
    dictionary = pad.get("dictionary") or cfg["aruco"]["dictionary"]
    mat_w, mat_h = (float(v) for v in pad["mat_mm"])
    markers = []
    for m in pad["markers"]:
        size = float(m["size_mm"])
        markers.append({
            "id": int(m["id"]),
            "size": size,
            "margin": float(m.get("margin_mm", size / 6.0)),
            "cx": float(m["center_mm"][0]),
            "cy": float(m["center_mm"][1]),
        })
    cam = [float(v) for v in pad.get("camera_offset_mm", [0, 0])]
    check_layout(markers, mat_w, mat_h, cam)
    return dictionary, markers, mat_w, mat_h, cam


def box_mm(m):
    """Marker + white margin, pad frame (x0, y0, x1, y1)."""
    h = m["size"] / 2.0 + m["margin"]
    return m["cx"] - h, m["cy"] - h, m["cx"] + h, m["cy"] + h


def check_layout(markers, mat_w, mat_h, cam):
    ids = [m["id"] for m in markers]
    if len(ids) != len(set(ids)):
        raise SystemExit(f"duplicate marker ids in landing_pad: {ids} (each marker needs its own ID)")
    for m in markers:
        x0, y0, x1, y1 = box_mm(m)
        if x0 < -mat_w / 2 or x1 > mat_w / 2 or y0 < -mat_h / 2 or y1 > mat_h / 2:
            raise SystemExit(f"ID {m['id']} (with its margin) does not fit on the {mat_w:g}x{mat_h:g} mm mat")
        if m["margin"] < m["size"] / 6.0 - 1e-6 and m["margin"] < 5.0:
            print(f"WARNING: ID {m['id']} margin {m['margin']:g} mm is thinner than one cell "
                  f"({m['size'] / 6:.1f} mm) and under 5 mm: detection may fail on a busy background")
    for i, a in enumerate(markers):
        for b in markers[i + 1:]:
            ax0, ay0, ax1, ay1 = box_mm(a)
            bx0, by0, bx1, by1 = box_mm(b)
            if ax0 < bx1 and bx0 < ax1 and ay0 < by1 and by0 < ay1:
                raise SystemExit(f"IDs {a['id']} and {b['id']} overlap (including their white margins)")
    if not any(math.hypot(m["cx"] - cam[0], m["cy"] - cam[1]) < 5.0 for m in markers):
        print(f"WARNING: no marker is centred on camera_offset_mm {cam}: the camera will not see a "
              f"whole marker in the last centimetres of the landing")


# --------------------------------------------------------------------------- drawing
class Mat:
    """Raster of the whole mat. Pad frame (mm, y up) -> pixels (row down)."""

    def __init__(self, mat_w, mat_h, dpi):
        self.k = dpi / MM_PER_INCH                     # px per mm
        self.w_mm, self.h_mm = mat_w, mat_h
        self.img = np.full((round(mat_h * self.k), round(mat_w * self.k)), 255, np.uint8)

    def px(self, x_mm, y_mm):
        return (x_mm + self.w_mm / 2.0) * self.k, (self.h_mm / 2.0 - y_mm) * self.k

    def box_px(self, m):
        x0, y0, x1, y1 = box_mm(m)
        c0, r0 = self.px(x0, y1)
        c1, r1 = self.px(x1, y0)
        return round(c0), round(r0), round(c1), round(r1)


def put_text(img, text, x, y, h_mm, k, color=0):
    scale = h_mm * k / 22.0
    cv2.putText(img, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                max(1, round(h_mm * k / 12.0)), cv2.LINE_AA)


def draw_speckle(mat, markers, seed):
    """Random rotated rectangles and ellipses at 3 scales (corners for the flow tracker)."""
    rng = np.random.default_rng(seed)
    img, k = mat.img, mat.k
    img[:] = 225
    area = mat.w_mm * mat.h_mm
    #          (min, max size mm, fraction of the area covered)
    scales = [(14.0, 30.0, 0.08), (6.0, 14.0, 0.07), (2.5, 6.0, 0.04)]
    for lo, hi, cover in scales:                       # big first, small drawn on top
        n = int(cover * area / (((lo + hi) / 2.0) ** 2))
        for _ in range(n):
            cx, cy = rng.uniform(0, img.shape[1]), rng.uniform(0, img.shape[0])
            a, b = rng.uniform(lo, hi, 2) * k
            ang = rng.uniform(0, 180)
            tone = int(rng.choice([rng.integers(15, 90), rng.integers(120, 170), 250], p=[0.6, 0.25, 0.15]))
            if rng.random() < 0.7:
                pts = cv2.boxPoints(((cx, cy), (a, b), ang)).astype(np.int32)
                cv2.fillPoly(img, [pts], tone, cv2.LINE_AA)
            else:
                cv2.ellipse(img, (int(cx), int(cy)), (int(a / 2), int(b / 2)), ang, 0, 360, tone, -1, cv2.LINE_AA)

    # Registration crosses every 100 mm (check the tiles are butted straight with a ruler).
    arm, lw = round(4 * k), max(1, round(0.4 * k))
    for gx in np.arange(-math.floor(mat.w_mm / 200) * 100, mat.w_mm / 2, 100):
        for gy in np.arange(-math.floor(mat.h_mm / 200) * 100, mat.h_mm / 2, 100):
            c, r = (int(round(v)) for v in mat.px(gx, gy))
            cv2.rectangle(img, (c - arm - lw, r - arm - lw), (c + arm + lw, r + arm + lw), 255, -1)
            cv2.line(img, (c - arm, r), (c + arm, r), 0, lw)
            cv2.line(img, (c, r - arm), (c, r + arm), 0, lw)

    # White boxes where the markers are glued, outlined and labelled.
    for m in markers:
        c0, r0, c1, r1 = mat.box_px(m)
        img[r0:r1, c0:c1] = 255
        cv2.rectangle(img, (c0, r0), (c1 - 1, r1 - 1), 110, max(1, round(0.3 * k)))
        h = min(3.0, (c1 - c0) / k / 6.0)
        put_text(img, f"ID {m['id']}", c0 + 1.5 * k, (r0 + r1) / 2, h, k, 120)
        put_text(img, "^ FRONT", c0 + 1.5 * k, r0 + (h + 1.5) * k, h, k, 120)

    # Mat edge = trim line.
    cv2.rectangle(img, (0, 0), (img.shape[1] - 1, img.shape[0] - 1), 0, max(1, round(0.5 * k)))


def marker_image(dictionary, m, k):
    """Marker + white margin, snapped to whole pixels per cell. Returns (img, real_size_mm)."""
    cell = max(1, round(m["size"] / 6.0 * k))
    side = 6 * cell
    marker = cv2.aruco.generateImageMarker(get_dictionary(dictionary), m["id"], side)
    pad = round(m["margin"] * k)
    out = np.full((side + 2 * pad, side + 2 * pad), 255, np.uint8)
    out[pad:pad + side, pad:pad + side] = marker
    return out, side / k


def assembled(mat, markers, dictionary):
    """The mat as it looks after gluing (for the preview and the self-test)."""
    img = mat.img.copy()
    for m in markers:
        tile, _ = marker_image(dictionary, m, mat.k)
        c0, r0, c1, r1 = mat.box_px(m)
        img[r0:r1, c0:c1] = cv2.resize(tile, (c1 - c0, r1 - r0), interpolation=cv2.INTER_NEAREST)
    return img


# --------------------------------------------------------------------------- pages
def tile_pages(mat, margin_mm):
    k = mat.k
    cw, ch = A4_MM[0] - 2 * margin_mm, A4_MM[1] - 2 * margin_mm
    cols, rows = math.ceil(mat.w_mm / cw), math.ceil(mat.h_mm / ch)
    page_w, page_h = round(A4_MM[0] * k), round(A4_MM[1] * k)
    m_px = round(margin_mm * k)
    cw_px, ch_px = round(cw * k), round(ch * k)
    pages = []
    for r in range(rows):
        for c in range(cols):
            page = np.full((page_h, page_w), 255, np.uint8)
            x0, y0 = round(c * cw * k), round(r * ch * k)
            crop = mat.img[y0:y0 + ch_px, x0:x0 + cw_px]
            page[m_px:m_px + crop.shape[0], m_px:m_px + crop.shape[1]] = crop
            if crop.shape[1] < cw_px or crop.shape[0] < ch_px:   # past the mat edge
                put_text(page, "outside the mat: trim off", m_px + 5 * k,
                         m_px + ch_px - 5 * k, 3.0, k, 150)

            # Crop marks in the margin, in line with the content edges.
            L, g, lw = round(min(6.0, margin_mm - 1.5) * k), round(1.0 * k), max(1, round(0.25 * k))
            xs, ys = (m_px, m_px + cw_px), (m_px, m_px + ch_px)
            for x in xs:
                for y, d in ((ys[0], -1), (ys[1], 1)):
                    cv2.line(page, (x, y + d * g), (x, y + d * (g + L)), 0, lw)
            for y in ys:
                for x, d in ((xs[0], -1), (xs[1], 1)):
                    cv2.line(page, (x + d * g, y), (x + d * (g + L), y), 0, lw)

            th = min(2.5, margin_mm * 0.4)
            put_text(page, f"TILE row {r + 1}/{rows} col {c + 1}/{cols}  (row 1 = FRONT edge)"
                     f"  print 100%, trim on crop marks",
                     m_px + 2 * k, (margin_mm / 2 + th / 2) * k, th, k)
            # 100 mm scale bar in the bottom margin
            yb = page_h - round(margin_mm / 2 * k)
            xb = m_px + round(10 * k)
            cv2.line(page, (xb, yb), (xb + round(100 * k), yb), 0, lw)
            for t in range(11):
                xt = xb + round(t * 10 * k)
                cv2.line(page, (xt, yb), (xt, yb - round((2.0 if t % 5 == 0 else 1.0) * k)), 0, lw)
            put_text(page, "100 mm - check with a ruler", xb + 105 * k, yb, th, k)
            pages.append(page)
    return pages, rows, cols


def marker_pages(markers, dictionary, k):
    """Shelf-pack the markers (with margin) onto A4 pages."""
    page_w, page_h = round(A4_MM[0] * k), round(A4_MM[1] * k)
    edge, gap, label_h = round(12 * k), round(10 * k), round(9 * k)
    pages, page = [], None
    x = y = shelf = 0
    real = {}
    for m in sorted(markers, key=lambda m: -m["size"]):
        img, real_mm = marker_image(dictionary, m, k)
        real[m["id"]] = real_mm
        h, w = img.shape[0] + 2 * label_h, img.shape[1]
        if page is None or x + w > page_w - edge:
            x, y, shelf = edge, (y + shelf + gap) if page is not None else edge, 0
        if page is None or y + h > page_h - edge:
            page = np.full((page_h, page_w), 255, np.uint8)
            pages.append(page)
            x, y, shelf = edge, edge, 0
        top = y + label_h
        page[top:top + img.shape[0], x:x + w] = img
        # dashed cut line on the outer edge of the white margin
        dash, lw = max(2, round(1.5 * k)), max(1, round(0.25 * k))
        y_lo, y_hi, x_lo, x_hi = top, top + img.shape[0] - 1, x, x + w - 1
        for i in range(0, w, 2 * dash):
            for yy in (y_lo, y_hi):
                cv2.line(page, (x + i, yy), (x + min(i + dash, w - 1), yy), 80, lw)
        for i in range(0, img.shape[0], 2 * dash):
            for xx in (x_lo, x_hi):
                cv2.line(page, (xx, top + i), (xx, top + min(i + dash, img.shape[0] - 1)), 80, lw)
        th = 2.6
        put_text(page, f"^ FRONT   ID {m['id']}", x, top - 2.5 * k, th, k)
        put_text(page, f"{real_mm:.1f} mm square", x, top + img.shape[0] + 4 * k, th, k)
        put_text(page, "cut on dashed line", x, top + img.shape[0] + 7.5 * k, th, k)
        x += w + gap
        shelf = max(shelf, h)
    return pages, real


def save_pdf(pages, path, dpi):
    from PIL import Image
    ims = [Image.fromarray(p) for p in pages]
    ims[0].save(path, save_all=True, append_images=ims[1:], resolution=dpi)


# --------------------------------------------------------------------------- self-test
def self_test(img, mat, markers, dictionary, cam):
    """Top-down pinhole view of the assembled mat, camera above camera_offset_mm."""
    det = ArucoDetector(dictionary, ids=None)
    feat = dict(maxCorners=75, qualityLevel=0.015, minDistance=8, blockSize=7)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))
    print("\nSelf-test (simulated top-down view, camera above the touchdown point):")
    print("  height | markers detected          | flow features (of 75) in a 160 px frame")
    for h in (0.12, 0.25, 0.35, 0.5, 1.0, 1.5, 2.0):
        s = CAM_F_PX / (h * 1000.0) / mat.k                # image px per mat px
        big = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        cc, cr = (v * s for v in mat.px(*cam))
        view = np.full((CAM_H, CAM_W), 128, np.uint8)       # outside the mat: grey floor
        x0, y0 = int(round(cc - CAM_W / 2)), int(round(cr - CAM_H / 2))
        sx0, sy0 = max(0, x0), max(0, y0)
        sx1, sy1 = min(big.shape[1], x0 + CAM_W), min(big.shape[0], y0 + CAM_H)
        if sx1 > sx0 and sy1 > sy0:
            view[sy0 - y0:sy1 - y0, sx0 - x0:sx1 - x0] = big[sy0:sy1, sx0:sx1]
        view = cv2.GaussianBlur(view, (3, 3), 0.8)
        found = sorted(i for i, _ in det.detect(view))
        small = clahe.apply(cv2.resize(view, (FLOW_WORK_W, round(CAM_H * FLOW_WORK_W / CAM_W)),
                                       interpolation=cv2.INTER_AREA))
        pts = cv2.goodFeaturesToTrack(small, mask=None, **feat)
        n = 0 if pts is None else len(pts)
        print(f"  {h:4.2f} m | {str(found):25s} | {n}")
    print("  (ideal images; real ones add blur, glare and perspective. Expect ID 20 up to ~0.6 m,"
          " ID 10 from ~0.33 m)")


# --------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--margin-mm", type=float, default=8.0,
                    help="unprintable border of your printer (default 8 mm)")
    ap.add_argument("--dpi", type=int, default=300, help="tiles resolution (markers use 600)")
    ap.add_argument("--seed", type=int, default=7, help="speckle pattern seed (same seed = same mat)")
    ap.add_argument("--no-test", action="store_true")
    args = ap.parse_args()

    dictionary, markers, mat_w, mat_h, cam = load_pad(args.config)
    OUT_DIR.mkdir(exist_ok=True)

    mat = Mat(mat_w, mat_h, args.dpi)
    draw_speckle(mat, markers, args.seed)
    tiles, rows, cols = tile_pages(mat, args.margin_mm)
    save_pdf(tiles, OUT_DIR / "pad_tiles.pdf", args.dpi)

    mk_dpi = 600
    mpages, real = marker_pages(markers, dictionary, mk_dpi / MM_PER_INCH)
    save_pdf(mpages, OUT_DIR / "pad_markers.pdf", mk_dpi)

    full = assembled(mat, markers, dictionary)
    prev_scale = 1500.0 / max(full.shape)
    cv2.imwrite(str(OUT_DIR / "pad_preview.png"),
                cv2.resize(full, None, fx=prev_scale, fy=prev_scale, interpolation=cv2.INTER_AREA))

    print(f"Mat {mat_w:g} x {mat_h:g} mm, dictionary {dictionary}, camera offset {cam} mm")
    print(f"  {OUT_DIR / 'pad_tiles.pdf'}: {rows} rows x {cols} cols = {len(tiles)} A4 tiles "
          f"(printer margin {args.margin_mm:g} mm)")
    print(f"  {OUT_DIR / 'pad_markers.pdf'}: {len(mpages)} page(s)")
    for m in markers:
        print(f"     ID {m['id']:>2}: {real[m['id']]:.2f} mm printed square (design {m['size']:g}), "
              f"margin {m['margin']:g} mm, centre ({m['cx']:g}, {m['cy']:g}) mm")
    print(f"  {OUT_DIR / 'pad_preview.png'}")

    if not args.no_test:
        self_test(full, mat, markers, dictionary, cam)


if __name__ == "__main__":
    main()
