#!/usr/bin/env python3
"""Live camera view in a web browser, for focusing and checking the image.

Serves the camera as MJPEG with an overlay showing a focus score (higher is
sharper) and any detected ArUco markers. Do not run with sudo.

    python3 tools/camera/live_view.py            # auto exposure, for focusing
    python3 tools/camera/live_view.py --fixed    # flight settings, for blur checks
    python3 tools/camera/live_view.py --stock    # stock tuning, to compare colour

Then open http://<pi-address>:8000 and press Ctrl+C here to stop. The stream
has no password, so only run it on a network you trust.
"""
import argparse
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
from picamera2 import Picamera2

from capture_fps import SENSOR_MODE, build_detector
from capture_test import TUNING

PAGE = b"""<!doctype html><html><head><title>UAV camera</title>
<style>body{margin:0;background:#111;display:flex;justify-content:center;
align-items:center;height:100vh}img{max-width:100%;max-height:100vh}</style>
</head><body><img src="/stream"></body></html>"""


class LatestFrame:
    """Holds the newest JPEG; each viewer waits for the next one."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0

    def publish(self, jpeg):
        with self.cond:
            self.jpeg = jpeg
            self.seq += 1
            self.cond.notify_all()

    def wait_next(self, last_seq):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout=2.0)
            return self.jpeg, self.seq


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--fps", type=float, help="frame rate (default: 60 fixed, 30 auto)")
    p.add_argument("--fixed", action="store_true", help="use fixed exposure and gain")
    p.add_argument("--exposure", type=int, default=4000, help="fixed exposure in us")
    p.add_argument("--gain", type=float, default=6.0, help="fixed analogue gain")
    p.add_argument("--tuning", type=Path, default=TUNING, help="tuning file to load")
    p.add_argument("--stock", action="store_true", help="use the stock tuning instead")
    p.add_argument("--dict", default="DICT_4X4_50", help="cv2.aruco dictionary name")
    p.add_argument("--quality", type=int, default=80, help="JPEG quality")
    return p.parse_args()


def draw_label(frame, text):
    cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
    cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)


def capture_loop(picam2, detector, latest, quality, stop):
    encode_args = [cv2.IMWRITE_JPEG_QUALITY, quality]
    fps, count, window_start = 0.0, 0, time.perf_counter()
    while not stop.is_set():
        # RGB888 in Picamera2 is BGR byte order: usable by OpenCV as is.
        frame = picam2.capture_array("main")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        focus = cv2.Laplacian(gray, cv2.CV_64F).var()
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        count += 1
        now = time.perf_counter()
        if now - window_start >= 1.0:
            fps, count, window_start = count / (now - window_start), 0, now

        markers = "none" if ids is None else ",".join(map(str, ids.ravel()))
        draw_label(frame, f"focus {focus:.0f} | {fps:.0f} fps | markers: {markers}")
        ok, jpeg = cv2.imencode(".jpg", frame, encode_args)
        if ok:
            latest.publish(jpeg.tobytes())


def make_handler(latest):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path == "/stream":
                self.stream()
            else:
                self.send_error(404)

        def stream(self):
            self.send_response(200)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            seq = 0
            try:
                while True:
                    jpeg, new_seq = latest.wait_next(seq)
                    if new_seq == seq:
                        continue
                    seq = new_seq
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                        + jpeg + b"\r\n"
                    )
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    args = parse_args()
    detector = build_detector(args.dict)

    fps = args.fps or (60.0 if args.fixed else 30.0)
    frame_us = int(1_000_000 / fps)
    controls = {"FrameDurationLimits": (frame_us, frame_us)}
    if args.fixed:
        controls.update(AeEnable=False, ExposureTime=args.exposure, AnalogueGain=args.gain)

    tuning = None
    if not args.stock:
        if not args.tuning.exists():
            sys.exit(f"Tuning file not found: {args.tuning}\n"
                     "Create it with: tools/camera/capture_test.py --make-tuning")
        tuning = Picamera2.load_tuning_file(str(args.tuning.resolve()))

    picam2 = Picamera2(tuning=tuning)
    picam2.configure(picam2.create_video_configuration(
        raw={"size": SENSOR_MODE},
        main={"size": (640, 480), "format": "RGB888"},
        controls=controls,
    ))
    picam2.start()

    latest = LatestFrame()
    stop = threading.Event()
    worker = threading.Thread(
        target=capture_loop, args=(picam2, detector, latest, args.quality, stop), daemon=True
    )
    worker.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(latest))
    server.daemon_threads = True
    print(f"Tuning: {'stock' if args.stock else args.tuning}", flush=True)
    print(f"Live view on port {args.port}. Press Ctrl+C to stop.", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        worker.join(timeout=2.0)
        server.server_close()
        picam2.stop()
        picam2.close()


if __name__ == "__main__":
    main()
