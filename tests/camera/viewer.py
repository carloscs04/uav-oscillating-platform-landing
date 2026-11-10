"""
viewer.py - show live frames either in an OpenCV window or in a web browser.

Over SSH there is no display, and cv2.imshow() aborts the whole program.
make_viewer() picks the right viewer automatically:
  * a desktop session (DISPLAY / WAYLAND_DISPLAY set) -> OpenCV window
  * otherwise -> MJPEG stream at http://<pi-ip>:<port>/

Both viewers have the same API, including keyboard input, so tools work
the same either way:

    viewer = make_viewer("preview", keys={"s": "save", "q": "quit"})
    while True:
        key = viewer.show(frame)      # returns a pressed key or None
        if key == "q": break
    viewer.close()
"""

import os
import queue
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2


def has_display():
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def make_viewer(title, mode="auto", port=8000, keys=None, max_width=1280):
    """mode: 'auto', 'window' or 'web'. keys: {'s': 'save', ...} shown as help."""
    if mode == "auto":
        mode = "window" if has_display() else "web"
    if mode == "window":
        return WindowViewer(title, keys)
    if mode == "web":
        return WebViewer(title, keys, port=port, max_width=max_width)
    raise ValueError(f"unknown viewer mode '{mode}'")


class WindowViewer:
    def __init__(self, title, keys=None):
        self.title = title
        if keys:
            print("Keys: " + ", ".join(f"{k} = {v}" for k, v in keys.items()))

    def show(self, frame):
        cv2.imshow(self.title, frame)
        code = cv2.waitKey(1) & 0xFF
        if code == 255:
            return None
        return "q" if code == 27 else chr(code)  # Esc behaves like q

    def close(self):
        cv2.destroyWindow(self.title)


class WebViewer:
    def __init__(self, title, keys=None, port=8000, max_width=1280, jpeg_quality=80):
        self.title = title
        self.keys = dict(keys or {})
        self.max_width = max_width
        self.jpeg_quality = jpeg_quality
        self._jpeg = None
        self._frame_id = 0
        self._clients = 0
        self._cond = threading.Condition()
        self._keys = queue.Queue()
        self._running = True

        self._server = ThreadingHTTPServer(("0.0.0.0", port), self._make_handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        print(f"Live view: http://{_local_ip()}:{port}/  (open in a browser on the same network)")
        if self.keys:
            print("Keys (press them in the browser tab): "
                  + ", ".join(f"{k} = {v}" for k, v in self.keys.items()))

    def show(self, frame):
        # Only spend CPU on JPEG encoding while someone is watching.
        if self._clients > 0:
            if self.max_width and frame.shape[1] > self.max_width:
                scale = self.max_width / frame.shape[1]
                frame = cv2.resize(frame, None, fx=scale, fy=scale,
                                   interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                with self._cond:
                    self._jpeg = buf.tobytes()
                    self._frame_id += 1
                    self._cond.notify_all()
        try:
            return self._keys.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self._running = False
        with self._cond:
            self._cond.notify_all()
        self._server.shutdown()
        self._server.server_close()

    # ------------------------------------------------------------- HTTP
    def _page(self):
        buttons = "".join(
            f'<button onclick="send(\'{k}\')"><b>{k}</b> {v}</button>'
            for k, v in self.keys.items())
        keylist = "".join(self.keys)
        return f"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{self.title}</title><style>
body{{margin:0;background:#111;color:#ddd;font:14px sans-serif;text-align:center}}
img{{max-width:100%;max-height:85vh;display:block;margin:8px auto}}
button{{margin:4px;padding:8px 14px;font-size:15px;background:#333;color:#eee;
border:1px solid #555;border-radius:6px;cursor:pointer}}</style></head><body>
<div style="padding:6px">{self.title}</div><img src="/stream"><div>{buttons}</div>
<script>
function send(k){{fetch('/key?k='+encodeURIComponent(k));}}
document.addEventListener('keydown',e=>{{
  const k=e.key==='Escape'?'q':e.key;
  if('{keylist}'.includes(k)){{send(k);e.preventDefault();}}
}});
</script></body></html>"""

    def _make_handler(viewer):
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass  # keep the terminal clean

            def do_GET(self):
                url = urlparse(self.path)
                if url.path == "/":
                    body = viewer._page().encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif url.path == "/key":
                    k = parse_qs(url.query).get("k", [""])[0][:1]
                    if k:
                        viewer._keys.put(k)
                    self.send_response(204)
                    self.end_headers()
                elif url.path == "/stream":
                    self._stream()
                else:
                    self.send_error(404)

            def _stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                viewer._clients += 1
                last_id = -1
                try:
                    while viewer._running:
                        with viewer._cond:
                            viewer._cond.wait_for(
                                lambda: viewer._frame_id != last_id or not viewer._running,
                                timeout=1.0)
                            jpeg, last_id = viewer._jpeg, viewer._frame_id
                        if jpeg is None or not viewer._running:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                                         + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    viewer._clients -= 1

        return Handler


def _local_ip():
    """IP address other machines can reach this Pi on (no packets are sent)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except OSError:
        return socket.gethostname()
    finally:
        s.close()
