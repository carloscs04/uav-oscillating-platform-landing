"""
aruco_live.py - live ArUco distance measurement with the active camera.

    python3 tests/vision/aruco_live.py                       # active_camera from config.yaml
    python3 tests/vision/aruco_live.py --camera usb_0bdc_8088
    python3 tests/vision/aruco_live.py --nominal-focal 1357  # no calibration yet (approximate!)

Shows each marker with its axes, ID, distance and quality. Opens a window on
the Pi desktop, or over SSH a browser stream at http://<pi-ip>:8000/.

Keys:
  m  measure: average every marker over 1 s, print it and append a row to
     tests/vision/measurements.csv (fill in true_distance_m with the tape measure)
  r  start / stop recording frames + detections to tests/vision/recordings/
  s  save a snapshot (raw frame)
  g  black & white view
  q  quit

Distances are from the camera's optical centre (inside the lens) to the marker
centre. `z` is the part straight ahead of the lens; `distance` is the full
straight line.
"""

import argparse
import csv
import logging
import queue
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

import _paths  # noqa: F401
from calibration import CalibrationNotFound, load_calibration
from cameras import load_config, open_camera
from src.logic.vision import ArucoDetector, CameraModel, estimate_all
from viewer import make_viewer

HERE = Path(__file__).resolve().parent
MEASUREMENTS_CSV = HERE / "measurements.csv"
RECORDINGS_DIR = HERE / "recordings"
SNAPSHOT_DIR = HERE / "snapshots"
MEASURE_WINDOW_S = 1.0


class Recorder:
    """Writes frames (JPEG q95) + one CSV row per detection in a background
    thread, so recording does not slow the measurement loop."""

    def __init__(self, cam, model, acfg):
        self.dir = RECORDINGS_DIR / f"{cam.calib_key}_{time.strftime('%Y%m%d_%H%M%S')}"
        self.dir.mkdir(parents=True)
        with open(self.dir / "session.yaml", "w") as f:
            yaml.safe_dump({"camera": cam.info(), "aruco": dict(acfg),
                            "calibrated": model.calibrated,
                            "camera_matrix": model.camera_matrix.tolist(),
                            "dist_coeffs": model.dist_coeffs.ravel().tolist()}, f, sort_keys=False)
        self._csv_file = open(self.dir / "detections.csv", "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["frame", "stamp", "id", "x", "y", "z", "distance",
                            "reproj_err_px", "ambiguity", "view_angle_deg", "marker_px", "reliable",
                            "c0x", "c0y", "c1x", "c1y", "c2x", "c2y", "c3x", "c3y"])
        self._q = queue.Queue(maxsize=120)
        self.frames = self.dropped = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def add(self, frame, stamp, measurements):
        try:
            self._q.put_nowait((self.frames, frame, stamp, measurements))
            self.frames += 1
        except queue.Full:
            self.dropped += 1

    def _run(self):
        while True:
            item = self._q.get()
            if item is None:
                break
            idx, frame, stamp, ms = item
            cv2.imwrite(str(self.dir / f"frame_{idx:06d}.jpg"), frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 95])
            for m in ms:
                self._csv.writerow([idx, f"{stamp:.6f}", m.marker_id, f"{m.x:.5f}", f"{m.y:.5f}",
                                    f"{m.z:.5f}", f"{m.distance:.5f}", f"{m.reproj_err_px:.3f}",
                                    f"{m.ambiguity:.3f}", f"{m.view_angle_deg:.1f}",
                                    f"{m.marker_px:.1f}", int(m.reliable),
                                    *[f"{v:.2f}" for v in m.corners.ravel()]])

    def close(self):
        self._q.put(None)
        self._thread.join()
        self._csv_file.close()
        print(f"Recording saved: {self.dir}  ({self.frames} frames, {self.dropped} dropped)")


def summarise(samples, cam, model):
    """samples: {id: [MarkerMeasurement]} -> print + append to measurements.csv."""
    if not samples:
        print("measure: no markers seen during the window")
        return
    new_file = not MEASUREMENTS_CSV.exists()
    with open(MEASUREMENTS_CSV, "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["time", "camera", "calibrated", "id", "n", "distance_m", "std_mm",
                        "x_m", "y_m", "z_m", "view_angle_deg", "reproj_err_px", "marker_px",
                        "true_distance_m", "notes"])
        print("\n=== measurement (1 s average) ===")
        for marker_id, ms in sorted(samples.items()):
            d = np.array([m.distance for m in ms])
            t = np.array([m.tvec for m in ms]).mean(axis=0)
            ang = np.mean([m.view_angle_deg for m in ms])
            err = np.mean([m.reproj_err_px for m in ms])
            px = np.mean([m.marker_px for m in ms])
            print(f"ID {marker_id:3d}: distance {d.mean():.4f} m  (+/-{d.std() * 1000:.1f} mm, "
                  f"n={len(d)})   x {t[0]:+.3f}  y {t[1]:+.3f}  z {t[2]:.3f}   "
                  f"angle {ang:.0f} deg   err {err:.2f} px")
            w.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), cam.calib_key, int(model.calibrated),
                        marker_id, len(d), f"{d.mean():.4f}", f"{d.std() * 1000:.2f}",
                        f"{t[0]:.4f}", f"{t[1]:.4f}", f"{t[2]:.4f}", f"{ang:.1f}",
                        f"{err:.3f}", f"{px:.1f}", "", ""])
    print(f"appended to {MEASUREMENTS_CSV} (fill in true_distance_m)\n")


def draw(view, measurements, model, sizes, scale):
    """Outline (green = reliable, red = failed quality check), axes (x red,
    y green, z blue, half the marker size long) and the numbers."""
    thick = max(1, int(2 * scale))
    for m in measurements:
        color = (0, 255, 0) if m.reliable else (0, 0, 255)
        cv2.polylines(view, [m.corners.astype(np.int32).reshape(-1, 1, 2)], True, color, thick)
        cv2.drawFrameAxes(view, model.camera_matrix, model.dist_coeffs, m.rvec, m.tvec,
                          sizes[m.marker_id] / 2, thick)
        x0, y0 = m.corners[:, 0].min(), m.corners[:, 1].min()
        lines = [f"ID {m.marker_id}  {m.distance:.3f} m",
                 f"x{m.x:+.2f} y{m.y:+.2f} z{m.z:.2f}",
                 f"{m.view_angle_deg:.0f}deg  err {m.reproj_err_px:.2f}px"]
        for k, text in enumerate(lines):
            org = (int(x0), int(y0 - (len(lines) - k) * 26 * scale - 6))
            cv2.putText(view, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7 * scale, (0, 0, 0),
                        int(5 * scale), cv2.LINE_AA)
            cv2.putText(view, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7 * scale, color,
                        thick, cv2.LINE_AA)


def hud(view, lines, scale):
    y = int(30 * scale)
    for text, color in lines:
        cv2.putText(view, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * scale, (0, 0, 0),
                    int(5 * scale), cv2.LINE_AA)
        cv2.putText(view, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * scale, color,
                    max(1, int(2 * scale)), cv2.LINE_AA)
        y += int(30 * scale)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", help="camera name from config.yaml (default: active_camera)")
    ap.add_argument("--config", help="path to config.yaml")
    ap.add_argument("--nominal-focal", type=float,
                    help="run without calibration using this focal length in px (approximate)")
    ap.add_argument("--refine", choices=ArucoDetector.REFINE_METHODS, default="edges")
    ap.add_argument("--view", choices=["auto", "window", "web"], default="auto")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    config = load_config(args.config)
    acfg = config["aruco"]
    sizes = {int(i): float(acfg["marker_mm"]) / 1000.0 for i in (acfg.get("ids") or range(1000))}
    sizes.update({int(k): float(v) / 1000.0 for k, v in (acfg.get("sizes_mm") or {}).items()})
    max_err = float(acfg.get("max_reproj_err_px", 2.0))
    detector = ArucoDetector(acfg["dictionary"], acfg.get("ids"), refine=args.refine)

    cam = open_camera(args.camera, config)
    try:
        model = CameraModel.from_calibration(load_calibration(cam.calib_key), cam.camera_id)
    except CalibrationNotFound as exc:
        if not args.nominal_focal:
            cam.close()
            raise SystemExit(f"{exc}\nOr run with --nominal-focal <px> for approximate distances.")
        model = CameraModel.nominal(cam.resolution, args.nominal_focal, cam.camera_id)
        print(f"WARNING: no calibration, using nominal focal {args.nominal_focal:g} px")

    print(f"Camera {cam.calib_key}   calibrated={model.calibrated}   "
          f"markers {acfg['dictionary']} ids={acfg.get('ids') or 'all'} "
          f"size {acfg['marker_mm']:g} mm   refine={args.refine}")
    viewer = make_viewer(f"aruco live - {cam.calib_key}", mode=args.view, port=args.port,
                         keys={"m": "measure 1 s", "r": "record", "s": "snapshot",
                               "g": "black & white", "q": "quit"})

    scale = max(0.7, cam.resolution[0] / 1600)
    gray_view, recorder = False, None
    measure_until, samples = 0.0, {}
    fps, last = 0.0, time.monotonic()
    proc_ms = 0.0
    try:
        while True:
            ok, frame, stamp = cam.read()
            if not ok:
                print("frame grab failed")
                break
            t0 = time.perf_counter()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ms = estimate_all(detector.detect(gray), sizes, model, stamp, max_err)
            proc_ms = 0.9 * proc_ms + 0.1 * (time.perf_counter() - t0) * 1000
            fps = 0.9 * fps + 0.1 / max(stamp - last, 1e-6)
            last = stamp

            if recorder:
                recorder.add(frame, stamp, ms)
            if measure_until:
                for m in ms:
                    if m.reliable:
                        samples.setdefault(m.marker_id, []).append(m)
                if stamp >= measure_until:
                    summarise(samples, cam, model)
                    measure_until, samples = 0.0, {}

            view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) if gray_view else frame.copy()
            draw(view, ms, model, sizes, scale)
            status = []
            if recorder:
                status.append(f"REC {recorder.frames}")
            if measure_until:
                status.append("MEASURING...")
            hud(view, [
                (f"{cam.calib_key}  {'calibrated' if model.calibrated else 'NOT CALIBRATED'}",
                 (255, 255, 255) if model.calibrated else (0, 0, 255)),
                (f"{fps:.0f} FPS   processing {proc_ms:.0f} ms   markers {len(ms)}",
                 (255, 255, 255)),
                ("   ".join(status), (0, 200, 255)),
            ], scale)

            key = viewer.show(view)
            if key == "q":
                break
            elif key == "g":
                gray_view = not gray_view
            elif key == "m" and not measure_until:
                measure_until, samples = stamp + MEASURE_WINDOW_S, {}
                print("measuring for 1 s ... hold still")
            elif key == "r":
                if recorder:
                    recorder.close()
                    recorder = None
                else:
                    recorder = Recorder(cam, model, acfg)
                    print(f"recording to {recorder.dir}")
            elif key == "s":
                SNAPSHOT_DIR.mkdir(exist_ok=True)
                path = SNAPSHOT_DIR / f"{cam.calib_key}_{int(time.time())}.png"
                cv2.imwrite(str(path), frame)
                print(f"Saved {path}")
    except KeyboardInterrupt:
        pass
    finally:
        if recorder:
            recorder.close()
        viewer.close()
        cam.close()


if __name__ == "__main__":
    main()
