import cv2
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from .telemetry_hub import TelemetryHub

_current_frame = None
_frame_lock = threading.Lock()


class _ThreadedServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class _MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            while True:
                with _frame_lock:
                    if _current_frame is None:
                        continue
                    _, jpeg = cv2.imencode(".jpg", _current_frame)
                    frame_bytes = jpeg.tobytes()

                try:
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-type", "image/jpeg")
                    self.send_header("Content-length", str(len(frame_bytes)))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b"\r\n")
                except Exception:
                    break


class VideoStreamer:
    """Self-contained MJPEG video server."""

    def __init__(self, hub: TelemetryHub):
        self.port = hub.config["video_port"]
        self.server = None
        self._thread = None

    def start(self):
        try:
            self.server = _ThreadedServer(("0.0.0.0", self.port), _MJPEGHandler)
            self._thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self._thread.start()
            print(f"[VideoStreamer] Video server running on port {self.port}")
        except Exception as e:
            print(f"[VideoStreamer] Error starting video server: {e}")

    def push_frame(self, frame):
        """Called by camera driver to update video stream."""
        global _current_frame
        if frame is not None:
            with _frame_lock:
                _current_frame = frame.copy()

    def stop(self):
        if self.server:
            self.server.shutdown()
            self.server.server_close()