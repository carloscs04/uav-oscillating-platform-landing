# File: src/inputs/camera_driver/master_camera.py

import base64
import os
import threading
import time
from typing import Any, Optional, Tuple

import cv2

from src.inputs.camera_driver.calib_params import CalibrationNotFound, load_calibration
from src.inputs.camera_driver.camera_state import CameraState
from src.inputs.camera_driver.cameras import load_config, open_camera

UDP_SAFE_B64_CHARS = 60000   # keep the video packet under the ~65 KB UDP limit


class CameraManager:
    """Main Facade API & Lifecycle Manager for the onboard camera.

    Owns CameraState (video preview + camera status, sent on port 5003) and
    shares every grey frame with consumers such as VisionManager.

        camera = CameraManager()
        camera.start()                         # background open / auto-reopen supervisor
        seq, gray, stamp = camera.wait_for_frame(last_seq)
        camera.export_telemetry_package()      # CameraState packets + node_health
        camera.stop()
    """

    def __init__(self, config_path: Optional[str] = None, camera: Optional[str] = None):
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "config.yaml")

        self.config_path = config_path
        self.config = self._load_config()
        self.camera_name = camera              # None = active_camera from config.yaml

        stream = self.config.get("stream", {})
        self.preview_width = int(stream.get("preview_width", 480))
        self.preview_quality = int(stream.get("preview_quality", 60))
        self.preview_period = 1.0 / max(1.0, float(stream.get("preview_hz", 15)))
        self.reopen_after = int(stream.get("reopen_after_failures", 30))

        self.cam: Optional[Any] = None
        self.connected: bool = False
        self.running: bool = False
        self.current_rate_hz: float = 0.0
        self._supervisor_thread: Optional[threading.Thread] = None

        # Camera intrinsics for consumers (VisionManager): calibration of the open camera.
        self.calibration = None                # calib_params.Calibration or None
        self.nominal_focal_px: Optional[float] = None
        self.camera_generation: int = 0        # +1 every time a (possibly different) camera opens

        # Latest frame, shared with consumers
        self._frame_cond = threading.Condition()
        self._frame_seq = 0
        self._frame_gray = None
        self._frame_stamp = 0.0

        self.state = CameraState()
        self.state.state = self.state          # tx/*.py compatibility (node.state)
        self._sync_state_link()

        self.error_count = 0
        self.last_error = ""

    def _load_config(self) -> dict:
        return load_config(self.config_path)

    # ------------------------------------------------------------- lifecycle
    def start(self, timeout: float = 3.0) -> bool:
        """Starts background open / auto-reopen supervisor loop."""
        if self.running:
            return self.connected

        self.running = True
        self._supervisor_thread = threading.Thread(
            target=self._camera_supervisor, name="CameraManager", daemon=True)
        self._supervisor_thread.start()

        start_wait = time.time()
        while time.time() - start_wait < timeout:
            if self.connected:
                return True
            time.sleep(0.1)
        return self.connected

    def stop(self):
        """Gracefully halts the supervisor and releases the camera."""
        self.running = False
        if self._supervisor_thread is not None:
            self._supervisor_thread.join(timeout=3.0)
            self._supervisor_thread = None
        self._cleanup_camera()
        self.state.update_frame("")
        with self._frame_cond:
            self._frame_cond.notify_all()
        print("[CAMERA] Node stopped cleanly.")

    def is_healthy(self) -> bool:
        return self.connected

    def export_telemetry_package(self) -> dict:
        """CameraState packets (video_frame, camera_status) + node_health."""
        package = self.state.export_telemetry_data()
        package["node_health"] = self.state.export_node_health(self.current_rate_hz, self.connected)
        return package

    # --------------------------------------------------------- frame sharing
    @property
    def calib_key(self) -> Optional[str]:
        return self.cam.calib_key if self.cam else None

    @property
    def resolution(self) -> Optional[Tuple[int, int]]:
        return self.cam.resolution if self.cam else None

    def wait_for_frame(self, last_seq: int = 0, timeout: float = 1.0):
        """Block until a frame newer than `last_seq` exists.
        Returns (seq, gray, stamp) or None on timeout / stop."""
        if not self.running:
            time.sleep(min(timeout, 0.2))      # camera stopped: don't let callers spin
            return None
        deadline = time.monotonic() + timeout
        with self._frame_cond:
            while self._frame_seq <= last_seq and self.running:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._frame_cond.wait(remaining)
            if self._frame_seq <= last_seq:
                return None
            return self._frame_seq, self._frame_gray, self._frame_stamp

    # ------------------------------------------------------------ internals
    def _sync_state_link(self):
        """Mirror link status on CameraState (Bridge_WIFI tx/camera.py reads it there)."""
        setattr(self.state, "connected", self.connected)
        setattr(self.state, "current_rate_hz", round(self.current_rate_hz, 1))

    def _open_camera(self):
        cam = open_camera(self.camera_name, self.config)
        cam_cfg = self.config["cameras"][cam.camera_id]
        try:
            self.calibration = load_calibration(cam.calib_key)
        except CalibrationNotFound:
            self.calibration = None
            print(f"[CAMERA Warning] {cam.calib_key} not calibrated: distances will use the "
                  f"nominal focal length ({cam_cfg.get('nominal_focal_px')} px).")
        self.nominal_focal_px = cam_cfg.get("nominal_focal_px")
        self.cam = cam
        self.camera_generation += 1
        self.connected = True
        self.current_rate_hz = 0.0
        w, h = cam.resolution
        self.state.update_frame("", fps=0.0, resolution=f"{w}x{h}")
        self._sync_state_link()
        print(f"[CAMERA] Connected (ONLINE): {cam.calib_key}, "
              f"calibrated={self.calibration is not None}")

    def _cleanup_camera(self):
        self.connected = False
        self.current_rate_hz = 0.0
        self._sync_state_link()
        if self.cam is not None:
            try:
                self.cam.close()
            except Exception:
                pass
            self.cam = None

    def _encode_preview(self, gray) -> str:
        h, w = gray.shape[:2]
        scale = min(1.0, self.preview_width / w)
        small = cv2.resize(gray, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        quality = self.preview_quality
        while True:
            ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, quality])
            data = base64.b64encode(buf.tobytes()).decode("ascii") if ok else ""
            if len(data) <= UDP_SAFE_B64_CHARS or quality <= 20:
                break
            quality -= 15
        return data if len(data) <= UDP_SAFE_B64_CHARS else ""

    def _camera_supervisor(self):
        """Background thread: open (with fallback), read frames, reopen on failure."""
        attempt_logged, retry_delay = False, 1.0
        failures, last_stamp, next_preview = 0, None, 0.0

        while self.running:
            # 1. AUTO-RECONNECT: (re)open the camera
            if self.cam is None:
                if not attempt_logged:
                    print("[CAMERA] Connecting...")
                    attempt_logged = True
                try:
                    self._open_camera()
                    attempt_logged, retry_delay = False, 1.0
                    failures, last_stamp = 0, None
                except Exception as e:
                    self.last_error = str(e).splitlines()[0]
                    self.error_count += 1
                    self._cleanup_camera()
                    deadline = time.monotonic() + retry_delay
                    while self.running and time.monotonic() < deadline:
                        time.sleep(0.1)
                    retry_delay = min(retry_delay * 2, 10.0)
                    continue

            # 2. READ: one frame
            try:
                ok, frame, stamp = self.cam.read()
            except Exception as e:
                ok, frame, stamp = False, None, time.monotonic()
                self.last_error = str(e)
            if not ok:
                failures += 1
                self.error_count += 1
                # 3. WATCHDOG: too many failed reads -> reopen
                if failures >= self.reopen_after:
                    print("[CAMERA] Disconnected (no frames). Reconnecting...")
                    self._cleanup_camera()
                continue
            failures = 0

            gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if last_stamp is not None and stamp > last_stamp:
                inst = 1.0 / (stamp - last_stamp)
                self.current_rate_hz = inst if self.current_rate_hz == 0 else \
                    0.9 * self.current_rate_hz + 0.1 * inst
            last_stamp = stamp
            self._sync_state_link()

            # 4. SHARE: latest frame for consumers (VisionManager)
            with self._frame_cond:
                self._frame_seq += 1
                self._frame_gray, self._frame_stamp = gray, stamp
                self._frame_cond.notify_all()

            # 5. STREAM: preview -> CameraState (rate-limited, one UDP packet)
            if stamp >= next_preview:
                next_preview = stamp + self.preview_period
                self.state.update_frame(self._encode_preview(gray), fps=self.current_rate_hz)

        self._cleanup_camera()


if __name__ == "__main__":
    # Standalone check:  python3 -m src.inputs.camera_driver.master_camera
    manager = CameraManager()
    manager.start()
    try:
        while True:
            time.sleep(1.0)
            pkg = manager.export_telemetry_package()
            print(f"[CAMERA] {manager.calib_key} | {pkg['node_health']['status']} "
                  f"{pkg['node_health']['rate_hz']} Hz | status {pkg['camera_status']} | "
                  f"preview {len(pkg['video_frame']['data']) // 1024} KB b64")
    except KeyboardInterrupt:
        pass
    finally:
        manager.stop()
