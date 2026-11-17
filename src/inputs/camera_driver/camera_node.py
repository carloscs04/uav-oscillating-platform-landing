# File: src/inputs/camera_driver/camera_node.py
"""
Onboard camera node: real camera -> CameraState (video) + VisionState (ArUco pose).

It follows the State pattern used by the other subsystems: the node only
WRITES into the State objects; the existing transmitters read them.

    CameraState  <- update_frame(...)        -> CameraTransmitter  port 5003  [video_frame, camera_status]
    VisionState  <- update_aruco_pose(...)   -> VisionTransmitter  port 5006  [aruco_pose, status]

aruco_pose: x, y, z and distance_m in METRES, camera frame (x right, y down,
z out of the lens); roll/pitch/yaw in degrees, (0, 0, 0) = marker facing the
camera squarely. tracking_state: LOCKED (marker in this frame), HOLDING (lost
for less than vision.hold_s), SEARCHING (no marker).

MasterNode drives it:
    start camera -> node.start()             (opens the camera, streams video)
    start vision -> node.enable_vision(True) (ArUco processing on the same frames)
    stop vision  -> node.enable_vision(False)
    stop camera  -> node.stop()

Self-healing: if the camera cannot be opened or stops delivering frames it is
(re)opened in the background, with fallback to the spare camera, until stop().
"""

import base64
import threading
import time
from typing import Any, Dict, Optional

import cv2

from src.inputs.camera_driver.calib_params import CalibrationNotFound, load_calibration
from src.inputs.camera_driver.cameras import load_config, open_camera
from src.logic.vision import ArucoDetector, CameraModel, marker_attitude_deg
from src.logic.vision.tracker import TargetTracker

UDP_SAFE_B64_CHARS = 60000   # keep the video packet under the ~65 KB UDP limit


class CameraNode:
    """Runs the camera in a background thread and publishes into the State objects."""

    def __init__(self, camera_state, vision_state, config_path: Optional[str] = None,
                 camera: Optional[str] = None):
        self.camera_state = camera_state
        self.vision_state = vision_state
        self.config = load_config(config_path)
        self.camera_name = camera              # None = active_camera from config.yaml

        vcfg = self.config.get("vision", {})
        self.reopen_after = int(vcfg.get("reopen_after_failures", 30))
        self.preview_width = int(vcfg.get("preview_width", 480))
        self.preview_quality = int(vcfg.get("preview_quality", 60))
        self.preview_period = 1.0 / max(1.0, float(vcfg.get("preview_hz", 15)))

        self.cam = None
        self.model: Optional[CameraModel] = None
        self.tracker: Optional[TargetTracker] = None

        self.vision_enabled = False
        # Held while a frame's ArUco result is computed and published, so that
        # enable_vision(False) can never be overtaken by a frame already in flight.
        self._vision_lock = threading.Lock()
        self.is_running = False
        self._thread: Optional[threading.Thread] = None

        self.fps = 0.0
        self.proc_ms = 0.0
        self.frames = 0
        self.error_count = 0
        self.last_error = ""

    # ------------------------------------------------------------- lifecycle
    def start(self):
        if self.is_running:
            return
        self.is_running = True
        self._thread = threading.Thread(target=self._run, name="CameraNode", daemon=True)
        self._thread.start()

    def stop(self):
        self.is_running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._close_camera()
        self._set_link(camera=False)
        self.camera_state.update_frame("")
        self._publish_no_target("SEARCHING")

    def enable_vision(self, on: bool):
        """Turn ArUco processing on/off (the camera keeps streaming video)."""
        with self._vision_lock:
            self.vision_enabled = bool(on)
            setattr(self.vision_state, "connected", bool(on and self.cam is not None))
            if not on:
                setattr(self.vision_state, "current_rate_hz", 0.0)
                self._publish_no_target("SEARCHING")

    def get_health(self) -> Dict[str, Any]:
        return {
            "running": self.is_running,
            "camera": self.cam.calib_key if self.cam else None,
            "calibrated": bool(self.model and self.model.calibrated),
            "vision_enabled": self.vision_enabled,
            "fps": round(self.fps, 1),
            "proc_ms": round(self.proc_ms, 1),
            "frames": self.frames,
            "error_count": self.error_count,
            "last_error": self.last_error,
        }

    # ------------------------------------------------------------ internals
    def _set_link(self, camera: bool):
        setattr(self.camera_state, "connected", camera)
        setattr(self.vision_state, "connected", camera and self.vision_enabled)
        if not camera:
            setattr(self.camera_state, "current_rate_hz", 0.0)
            setattr(self.vision_state, "current_rate_hz", 0.0)

    def _publish_no_target(self, tracking_state):
        self.vision_state.update_aruco_pose(detected=False, tracking_state=tracking_state)

    def _open(self):
        cam = open_camera(self.camera_name, self.config)
        cam_cfg = self.config["cameras"][cam.camera_id]
        try:
            model = CameraModel.from_calibration(load_calibration(cam.calib_key), cam.camera_id)
        except CalibrationNotFound:
            focal = cam_cfg.get("nominal_focal_px")
            if not focal:
                cam.close()
                raise
            model = CameraModel.nominal(cam.resolution, float(focal), cam.camera_id)
            print(f"[CAMERA_NODE Warning] {cam.calib_key} not calibrated: using nominal "
                  f"focal {focal} px, distances are approximate.")

        acfg = self.config["aruco"]
        vcfg = self.config.get("vision", {})
        ids = acfg.get("ids") or list(range(1000))
        sizes = {int(i): float(acfg["marker_mm"]) / 1000.0 for i in ids}
        sizes.update({int(k): float(v) / 1000.0 for k, v in (acfg.get("sizes_mm") or {}).items()})
        self.tracker = TargetTracker(
            detector=ArucoDetector(acfg["dictionary"], acfg.get("ids")),
            camera_model=model, sizes_m=sizes,
            max_reproj_err_px=float(acfg.get("max_reproj_err_px", 2.0)),
            primary_ids=vcfg.get("primary_ids") or acfg.get("ids"),
            smoothing=float(vcfg.get("smoothing", 0.6)),
            max_jump_m=float(vcfg.get("max_jump_m", 0.30)),
            hold_s=float(vcfg.get("hold_s", 0.5)))
        self.cam, self.model = cam, model
        self._set_link(camera=True)
        w, h = cam.resolution
        self.camera_state.update_frame("", fps=0.0, resolution=f"{w}x{h}")
        print(f"[CAMERA_NODE] {cam.calib_key} open, calibrated={model.calibrated}, "
              f"markers {acfg['dictionary']} ids={acfg.get('ids')} size {acfg['marker_mm']:g} mm")

    def _close_camera(self):
        if self.cam is not None:
            try:
                self.cam.close()
            except Exception as e:  # noqa: BLE001
                print(f"[CAMERA_NODE Warning] Camera close error: {e}")
            self.cam = None

    def _encode_preview(self, gray):
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

    def _publish_target(self, stamp):
        t = self.tracker.target(stamp)
        if not t.get("detected"):
            self._publish_no_target("SEARCHING")
            return
        roll, pitch, yaw = marker_attitude_deg(t["rvec"])
        self.vision_state.update_aruco_pose(
            detected=True, target_id=int(t["id"]),
            x=t["x"], y=t["y"], z=t["z"],
            roll=roll, pitch=pitch, yaw=yaw,
            tracking_state="LOCKED" if t["in_view"] else "HOLDING")

    def _run(self):
        failures, last_stamp, retry_delay, next_preview = 0, None, 1.0, 0.0
        while self.is_running:
            if self.cam is None:
                try:
                    self._open()
                    failures, retry_delay, last_stamp = 0, 1.0, None
                except Exception as e:  # noqa: BLE001 - keep retrying, like the MAVLink node
                    self.last_error = str(e).splitlines()[0]
                    self.error_count += 1
                    self._set_link(camera=False)
                    print(f"[CAMERA_NODE Error] Cannot open camera: {self.last_error}. "
                          f"Retrying in {retry_delay:.0f} s.")
                    deadline = time.monotonic() + retry_delay
                    while self.is_running and time.monotonic() < deadline:
                        time.sleep(0.1)
                    retry_delay = min(retry_delay * 2, 10.0)
                    continue

            try:
                ok, frame, stamp = self.cam.read()
            except Exception as e:  # noqa: BLE001
                ok, frame, stamp = False, None, time.monotonic()
                self.last_error = str(e)
            if not ok:
                failures += 1
                self.error_count += 1
                if failures >= self.reopen_after:
                    print(f"[CAMERA_NODE Error] {failures} failed reads: reopening the camera.")
                    self._close_camera()
                    self._set_link(camera=False)
                continue
            failures = 0

            gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if last_stamp is not None and stamp > last_stamp:
                inst = 1.0 / (stamp - last_stamp)
                self.fps = inst if self.fps == 0 else 0.9 * self.fps + 0.1 * inst
            last_stamp = stamp
            self.frames += 1
            setattr(self.camera_state, "current_rate_hz", round(self.fps, 1))

            # ArUco target -> VisionState
            with self._vision_lock:
                if self.vision_enabled:
                    t0 = time.perf_counter()
                    try:
                        self.tracker.update(gray, stamp)
                        self._publish_target(stamp)
                        setattr(self.vision_state, "connected", True)
                        setattr(self.vision_state, "current_rate_hz", round(self.fps, 1))
                    except Exception as e:  # noqa: BLE001 - one bad frame must not kill the node
                        self.error_count += 1
                        self.last_error = f"vision: {e}"
                    ms = (time.perf_counter() - t0) * 1000.0
                    self.proc_ms = ms if self.proc_ms == 0 else 0.9 * self.proc_ms + 0.1 * ms

            # Video preview -> CameraState (rate-limited, sized for one UDP packet)
            if stamp >= next_preview:
                next_preview = stamp + self.preview_period
                self.camera_state.update_frame(self._encode_preview(gray), fps=self.fps)

        self._close_camera()
