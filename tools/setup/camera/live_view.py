#!/usr/bin/env python3
"""Step 2 of 7: focus the lens using a live view in a web browser.

Serves the camera as MJPEG with an overlay showing a focus score (higher is
sharper) and any detected ArUco markers. Turn the lens ring until the score
peaks on a detailed target, then lock the ring. Do not run with sudo.

    .venv/bin/python tools/setup/camera/live_view.py            # auto exposure, for focusing
    .venv/bin/python tools/setup/camera/live_view.py --fixed    # settings from camera.yaml
    .venv/bin/python tools/setup/camera/live_view.py --stock    # stock tuning, to compare colour

Then open http://<pi-address>:8000 and press Ctrl+C here to stop. The stream
has no password, so only run it on a network you trust.

Next: step 3, capture_test.py.
"""
import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

from camera_common import (
    add_camera_args, annotate, build_detector, format_ids, grab, load_settings,
    open_camera,
)

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
    add_camera_args(p)
    p.add_argument("--fixed", action="store_true", help="use exposure and gain from camera.yaml")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--quality", type=int, default=80, help="JPEG quality")
    return p.parse_args()


def capture_loop(picam2, detector, latest, quality, stop):
    encode_args = [cv2.IMWRITE_JPEG_QUALITY, quality]
    fps, count, window_start = 0.0, 0, time.perf_counter()
    while not stop.is_set():
        frame, _ = grab(picam2)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        focus = cv2.Laplacian(gray, cv2.CV_64F).var()
        corners, ids, _ = detector.detectMarkers(gray)

        count += 1
        now = time.perf_counter()
        if now - window_start >= 1.0:
            fps, count, window_start = count / (now - window_start), 0, now

        label = f"focus {focus:.0f} | {fps:.0f} fps | markers: {format_ids(ids)}"
        annotate(frame, corners, ids, label)
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
    settings = load_settings(args, auto_exposure=not args.fixed)
    detector = build_detector(settings)
    picam2 = open_camera(settings)

    latest = LatestFrame()
    stop = threading.Event()
    worker = threading.Thread(
        target=capture_loop, args=(picam2, detector, latest, args.quality, stop), daemon=True
    )
    worker.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(latest))
    server.daemon_threads = True
    print(f"Tuning: {settings['tuning_file'] or 'stock'}", flush=True)
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
