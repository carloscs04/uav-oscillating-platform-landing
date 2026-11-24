# File: src/inputs/camera_driver/camera_node.py
"""
Onboard camera node: real camera -> CameraState (video) + VisionState (ArUco pose & Optical Flow).
"""

import base64
import threading
import time
from typing import Any, Dict, Optional

import cv2
import numpy as np

from src.inputs.camera_driver.calib_params import CalibrationNotFound, load_calibration
from src.inputs.camera_driver.cameras import load_config, open_camera
from src.logic.vision import ArucoDetector, CameraModel, marker_attitude_deg, marker_object_points
from src.logic.vision.optical_flow import OpticalFlowEstimator
from src.logic.vision.pixel_target import pad_touchdown, pixel_target, tilt_compensate
from src.logic.control.landing_law import LandingLaw, LandingParams
from src.logic.vision.target_select import load_registry, select_target
from src.logic.vision.tracker import TargetTracker

UDP_SAFE_B64_CHARS = 60000  # keep the video packet under the ~65 KB UDP limit

# Families tried by the discovery scan (`arucoscan on`). Each one costs a full detection pass,
# so the scan runs only on request and at SCAN_HZ, never at camera rate.
SCAN_DICTS = ("DICT_APRILTAG_36h11", "DICT_5X5_1000", "DICT_6X6_1000", "DICT_7X7_1000",
              "DICT_ARUCO_ORIGINAL", "DICT_APRILTAG_25h9", "DICT_APRILTAG_16h5")
SCAN_HZ = 2.0          # full scan (all families): finds unknown markers of any family
SCAN_FAST_HZ = 10.0    # in between, only the families of known markers + the current target
SCAN_HOLD_S = 1.0      # a marker missed by one scan stays this long (no blinking outline)
TARGET_MAX_AGE_S = 0.15   # only a marker seen this recently can be the control target
KNOWN_HOLD_S = 0.5        # a known marker lost this recently: do not switch to an unknown one
# Printed tags are often slightly bent / seen through lens distortion: the default 0.03
# rejected a real AprilTag whose edges were not perfectly straight.
SCAN_POLY_ACCURACY = 0.05
# Close up a tag's cells are ~20 px: the default largest threshold window (23 px) is too
# small to binarise them and the tag is missed. Larger windows = a bit slower (own thread).
SCAN_THRESH_WIN_MAX = 53


class CameraNode:
    """Runs the camera in a background thread and publishes into the State objects."""

    def __init__(
        self,
        camera_state,
        vision_state,
        config_path: Optional[str] = None,
        camera: Optional[str] = None,
        height_state=None,
        vehicle_state=None,
    ):
        self.camera_state = camera_state
        self.vision_state = vision_state
        self.height_state = height_state  # DistanceState (lidar): z for the pixel target
        self.vehicle_state = vehicle_state  # MAVLink VehicleState: ATTITUDE for tilt compensation
        self.config = load_config(config_path)
        self.camera_name = camera  # None = active_camera from config.yaml

        vcfg = self.config.get("vision", {})
        self.reopen_after = int(vcfg.get("reopen_after_failures", 30))
        self.preview_width = int(vcfg.get("preview_width", 480))
        self.preview_quality = int(vcfg.get("preview_quality", 60))
        self.preview_period = 1.0 / max(1.0, float(vcfg.get("preview_hz", 15)))
        # Other marker families (AprilTag...) found by the background scan, on by default
        if hasattr(self.vision_state, "scan_any"):
            self.vision_state.scan_any = bool(vcfg.get("scan_any", True))

        self.cam = None
        self.model: Optional[CameraModel] = None
        self.tracker: Optional[TargetTracker] = None
        self.flow_estimator: Optional[OpticalFlowEstimator] = None

        self.vision_enabled = False
        self._vision_lock = threading.Lock()
        self.is_running = False
        self._thread: Optional[threading.Thread] = None

        self._scan_detectors = None
        self._scan_found = []      # [(label, corners, last_seen monotonic), ...]
        self._scan_seen = {}       # label -> (corners, last_seen), only touched by the scan thread
        self._next_scan = 0.0
        self._next_full_scan = 0.0
        self.scan_ms = 0.0

        # Registered markers (known_markers.yaml), order = priority. `markers reload` in the CLI.
        self.registry = []
        self.reload_registry()
        self._target_label = None
        # Landing pad layout (config landing_pad): pad-family marker id -> centre on the pad, mm.
        # Any of them then points at the touchdown point (no jump from ID 10 to ID 20).
        pad = self.config.get("landing_pad") or {}
        same_dict = pad.get("dictionary", self.config["aruco"]["dictionary"]) == self.config["aruco"]["dictionary"]
        self.pad_centers = {int(m["id"]): tuple(float(v) for v in m["center_mm"])
                            for m in (pad.get("markers") or []) if same_dict and "center_mm" in m}
        # Landing law in SHADOW mode: computed every frame, published, never sent to the Pixhawk.
        self.landing = LandingLaw(LandingParams.from_dict(self.config.get("landing_law")))
        self._known_seen = {}   # known label -> (centre px, side px, monotonic time last seen)

        self.fps = 0.0
        self.proc_ms = 0.0
        self.frames = 0
        self.error_count = 0
        self.last_error = ""

    def reload_registry(self, path=None):
        try:
            self.registry = load_registry(path)
        except Exception as e:  # noqa: BLE001
            print(f"[CAMERA_NODE Warning] known_markers.yaml not loaded: {e}")
        return list(self.registry)

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
        """Turn ArUco processing on/off (optical flow and video keep running while camera is active)."""
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
            "scan_ms": round(self.scan_ms, 1),
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
        if hasattr(self.vision_state, "update_overlay"):
            self.vision_state.update_overlay([], 1, 1)
        if hasattr(self.vision_state, "update_pixel_target"):
            self.vision_state.update_pixel_target(None)
            self._shadow_step()

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
            print(
                f"[CAMERA_NODE Warning] {cam.calib_key} not calibrated: using nominal "
                f"focal {focal} px, distances are approximate."
            )

        K = getattr(model, "K", None)
        if K is not None:
            fx, fy = float(K[0, 0]), float(K[1, 1])
        else:
            fx = float(getattr(model, "fx", cam_cfg.get("nominal_focal_px", 500.0)))
            fy = float(getattr(model, "fy", fx))

        flow_cfg = self.config.get("optical_flow", {})
        self.flow_estimator = OpticalFlowEstimator(
            native_resolution=cam.resolution,
            focal_px=(fx, fy),
            work_width=int(flow_cfg.get("work_width", 160)),
            swap_xy=bool(flow_cfg.get("swap_xy", False)),
            invert_x=bool(flow_cfg.get("invert_x", False)),
            invert_y=bool(flow_cfg.get("invert_y", False)),
        )

        acfg = self.config["aruco"]
        vcfg = self.config.get("vision", {})
        ids = acfg.get("ids") or list(range(1000))
        sizes = {int(i): float(acfg["marker_mm"]) / 1000.0 for i in ids}
        sizes.update({int(k): float(v) / 1000.0 for k, v in (acfg.get("sizes_mm") or {}).items()})
        self.tracker = TargetTracker(
            detector=ArucoDetector(acfg["dictionary"], acfg.get("ids")),
            camera_model=model,
            sizes_m=sizes,
            max_reproj_err_px=float(acfg.get("max_reproj_err_px", 2.0)),
            primary_ids=vcfg.get("primary_ids") or acfg.get("ids"),
            smoothing=float(vcfg.get("smoothing", 0.6)),
            max_jump_m=float(vcfg.get("max_jump_m", 0.30)),
            hold_s=float(vcfg.get("hold_s", 0.5)),
        )
        self.cam, self.model = cam, model
        self._set_link(camera=True)
        w, h = cam.resolution
        self.camera_state.update_frame("", fps=0.0, resolution=f"{w}x{h}")
        print(
            f"[CAMERA_NODE] {cam.calib_key} open, calibrated={model.calibrated}, "
            f"markers {acfg['dictionary']} ids={acfg.get('ids')} size {acfg['marker_mm']:g} mm"
        )

    def _close_camera(self):
        if self.flow_estimator is not None:
            self.flow_estimator.reset()
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
            detected=True,
            target_id=int(t["id"]),
            x=t["x"],
            y=t["y"],
            z=t["z"],
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            tracking_state="LOCKED" if t["in_view"] else "HOLDING",
        )

    def _marker_axes(self, corners, rvec=None, tvec=None, size_m=None):
        """Image points of the marker origin and of the tips of its X, Y, Z axes (half a side
        long). Without a known pose/size, solve it with size 1: the rotation (what the axes
        show) does not depend on the size, only the distance does."""
        if self.model is None:
            return None
        K, dist = self.model.camera_matrix, self.model.dist_coeffs
        if rvec is None or tvec is None or not size_m:
            size_m = 1.0
            obj = marker_object_points(size_m)
            ok, rvec, tvec = cv2.solvePnP(obj, np.asarray(corners, np.float64).reshape(4, 1, 2),
                                          K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                return None
        L = 0.5 * float(size_m)
        axis = np.array([[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, L]], np.float64)
        pts, _ = cv2.projectPoints(axis, np.asarray(rvec, np.float64), np.asarray(tvec, np.float64), K, dist)
        return pts.reshape(4, 2)

    def _publish_overlay(self, measurements, shape):
        """Every marker in this frame (pad family from the main detector, other families from
        the scan) -> labelled outlines + RGB axes in VisionState, drawn by the ground station
        (the Pi only sends a few numbers per marker, not an annotated image). Then the target
        is chosen (known markers by priority, else the nearest unknown) for the pixel target."""
        now = time.monotonic()
        family = self.config["aruco"]["dictionary"].replace("DICT_", "")
        cands, seen = [], set()  # (label, corners, axes, last_seen)
        for m in measurements:  # configured pad IDs: real pose -> axes with the true size
            if m.corners is None:
                continue
            seen.add(m.marker_id)
            cands.append((f"{family} #{m.marker_id}", m.corners,
                          self._marker_axes(m.corners, m.rvec, m.tvec, self.tracker.sizes_m.get(m.marker_id)), now))
        # Same family, any other ID (already decoded by the detector: no extra detection pass)
        cands += [(f"{family} #{i}", c, self._marker_axes(c), now)
                  for i, c in getattr(self.tracker.detector, "last_all", []) if i not in seen]
        if getattr(self.vision_state, "scan_any", False):
            cands += [(label, c, self._marker_axes(c), t) for label, c, t in self._scan_found]

        K = getattr(self.model, "camera_matrix", None)
        pp = (float(K[0][2]), float(K[1][2])) if K is not None else (shape[1] / 2.0, shape[0] / 2.0)
        f = (float(K[0][0]), float(K[1][1])) if K is not None else None
        cands = self._drop_phantoms(cands, now)
        # Control target: only fresh detections (held scan results are drawn, never followed),
        # and no switch to an unknown marker right after a known one was lost for a frame.
        fresh = [c for c in cands if now - c[3] <= TARGET_MAX_AGE_S]
        known_recent = any(now - t <= KNOWN_HOLD_S for _c, _s, t in self._known_seen.values())
        if known_recent and not any(c[0] in self.registry for c in fresh):
            fresh = []
        idx, known = select_target([(c[0], c[1]) for c in fresh], self.registry, pp, self._target_label)
        chosen = fresh[idx] if idx is not None else None
        self._target_label = chosen[0] if chosen else None
        rank = set(self.registry)
        items = [(label, c, ax, label in rank) for label, c, ax, _t in cands]
        self.vision_state.update_overlay(items, shape[1], shape[0], self._target_label, pp, f)
        self._publish_pixel_target(chosen, known, shape)

    def _drop_phantoms(self, cands, now):
        """A known marker misread by another dictionary in the scan (seen: ARUCO_ORIGINAL #193
        on top of the 4x4 #20) is not a second marker: drop unknown candidates whose centre is
        within one side of a known marker seen in the last KNOWN_HOLD_S."""
        for label, c, _ax, t in cands:
            if label in self.registry and now - t <= TARGET_MAX_AGE_S:
                p = np.asarray(c, np.float64).reshape(4, 2)
                side = float(np.mean(np.linalg.norm(p - np.roll(p, 1, axis=0), axis=1)))
                self._known_seen[label] = (p.mean(axis=0), side, t)
        recent = [(ctr, side) for ctr, side, t in self._known_seen.values() if now - t <= KNOWN_HOLD_S]
        out = []
        for cand in cands:
            if cand[0] not in self.registry:
                ctr = np.asarray(cand[1], np.float64).reshape(4, 2).mean(axis=0)
                if any(np.linalg.norm(ctr - kc) < max(side, 10.0) for kc, side in recent):
                    continue
            out.append(cand)
        return out

    def _lidar_height(self):
        st = self.height_state
        if st is None or getattr(st, "sensor_status", "OK") != "OK":
            return None
        h = float(getattr(st, "distance_m", 0.0) or 0.0)
        return h if h > 0.03 else None

    def _publish_pixel_target(self, chosen, known, shape):
        """Control measurement of the chosen marker: centre vs the optical axis (pixels -> cm
        with the lidar height), z = lidar. `follow` says whether the controller may go to it:
        always for a known marker, for an unknown one only after `followunknown yes`."""
        if chosen is None or self.model is None:
            self.vision_state.update_pixel_target(None)
            self._shadow_step()
            return
        label, corners, _ax, seen_at = chosen
        allowed = getattr(self.vision_state, "follow_unknown", None)
        approval = "known" if known else {None: "ask", True: "yes", False: "no"}[allowed]
        lidar = self._lidar_height()
        t = pixel_target(corners, self.model.camera_matrix, self.model.dist_coeffs, lidar)
        t.update(self._tilt_fields(t, lidar, corners, label))
        t.update({"detected": True, "source": label, "known": bool(known),
                  "follow": bool(known or allowed is True), "approval": approval,
                  "stamp": seen_at, "age_s": round(time.monotonic() - seen_at, 3),
                  "un": round(t["u_px"] / shape[1], 4), "vn": round(t["v_px"] / shape[0], 4)})
        self.vision_state.update_pixel_target(t)
        self._shadow_step()

    def _shadow_step(self):
        """Run the landing law on the current measurement (SHADOW: published only). Target =
        pad touchdown point if the marker is on the pad, else the marker itself; only fresh,
        tilt-compensated and allowed (`follow`) measurements count."""
        pt = self.vision_state.get_pixel_target()
        target = None
        if pt.get("detected") and pt.get("follow") and (pt.get("age_s") or 0) <= 0.2:
            if pt.get("pad_fwd_m") is not None:
                target = (pt["pad_fwd_m"], pt["pad_right_m"])
            elif pt.get("fwd_m") is not None:
                target = (pt["fwd_m"], pt["right_m"])
        h = pt.get("h_vert_m") if pt.get("h_vert_m") is not None else self._lidar_height()
        out = self.landing.step(target, h, time.monotonic())
        out["target"] = ("pad" if pt.get("pad_fwd_m") is not None else "marker") if target else None
        out["t"] = time.monotonic()     # GuidedSender: law output age
        sender = getattr(self, "guided_sender", None)
        out["mode"] = sender.mode_word() if sender is not None else "SHADOW"
        if sender is not None:
            last = sender.status()
            out["sent"] = int(bool(last.get("sent")))
            out["sent_v_fwd"], out["sent_v_right"], out["sent_v_down"] = (
                last.get("v_fwd"), last.get("v_right"), last.get("v_down"))
            out["sent_reason"] = last.get("reason")
        if hasattr(self.vision_state, "update_landing_shadow"):
            self.vision_state.update_landing_shadow(out)

    def _tilt_fields(self, t, lidar, corners=None, label=None):
        """Marker offset from the drone centre in the drone's forward/right directions:
        raw (camera only) and tilt-compensated with the Pixhawk ATTITUDE (if recent). For a
        pad marker, also the touchdown point (pad_fwd_m, pad_right_m)."""
        out = {"fwd_raw_m": None, "right_raw_m": None, "fwd_m": None, "right_m": None,
               "h_vert_m": None, "roll_deg": None, "pitch_deg": None, "att_age_s": None,
               "pad_fwd_m": None, "pad_right_m": None}
        if lidar is not None:
            out["fwd_raw_m"], out["right_raw_m"], _ = (round(v, 4) for v in
                                                       tilt_compensate(t["nx"], t["ny"], 0.0, 0.0, lidar))
        vs = self.vehicle_state
        if vs is None or not hasattr(vs, "get_packet_with_time"):
            return out
        att, att_t = vs.get_packet_with_time("ATTITUDE")
        if not att or att_t is None:
            return out
        age = time.time() - att_t
        roll, pitch = float(att.get("roll", 0.0)), float(att.get("pitch", 0.0))
        out.update({"roll_deg": round(float(np.degrees(roll)), 2), "pitch_deg": round(float(np.degrees(pitch)), 2),
                    "att_age_s": round(age, 3)})
        if lidar is not None and age < 0.3:
            f, r, h = tilt_compensate(t["nx"], t["ny"], roll, pitch, lidar)
            if f is not None:
                out.update({"fwd_m": round(f, 4), "right_m": round(r, 4), "h_vert_m": round(h, 4)})
            mid = self._pad_id(label)
            if f is not None and corners is not None and mid is not None:
                td = pad_touchdown(corners, self.pad_centers[mid], self.model.camera_matrix,
                                   self.model.dist_coeffs, roll, pitch, lidar)
                if td is not None:
                    out.update({"pad_fwd_m": round(td[0], 4), "pad_right_m": round(td[1], 4)})
        return out

    def _pad_id(self, label):
        """'4X4_50 #10' -> 10 if it is a marker of the configured landing pad, else None."""
        family = self.config["aruco"]["dictionary"].replace("DICT_", "")
        if not label or not label.startswith(family + " #"):
            return None
        mid = int(label.split("#")[1])
        return mid if mid in self.pad_centers else None

    def _scan_any(self, gray, stamp):
        """Start a discovery scan on a copy of this frame in its own thread (~25-40 ms for all
        families), so the camera loop and the optical flow never wait for it."""
        if stamp < self._next_scan:
            return
        worker = getattr(self, "_scan_thread", None)
        if worker is not None and worker.is_alive():
            return
        self._next_scan = stamp + 1.0 / SCAN_FAST_HZ
        if stamp >= self._next_full_scan:
            self._next_full_scan = stamp + 1.0 / SCAN_HZ
            families = None  # all
        else:
            # Known markers' families, plus the current target's, so the followed marker is
            # refreshed at 10 Hz even when it is an unknown one
            families = {label.split(" #")[0] for label in self.registry}
            if self._target_label:
                families.add(self._target_label.split(" #")[0])
        self._scan_thread = threading.Thread(target=self._scan_frame, args=(gray.copy(), families),
                                             name="ArucoScan", daemon=True)
        self._scan_thread.start()

    def _scan_frame(self, gray, families=None):
        """Find markers of the given families (None = all). The pad family is skipped: the main
        detector already finds every ID of it at camera rate."""
        if self._scan_detectors is None:
            self._scan_detectors = []
            for name in SCAN_DICTS:
                if hasattr(cv2.aruco, name):
                    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
                    params = cv2.aruco.DetectorParameters()
                    params.polygonalApproxAccuracyRate = SCAN_POLY_ACCURACY
                    params.adaptiveThreshWinSizeMax = SCAN_THRESH_WIN_MAX
                    self._scan_detectors.append((name.replace("DICT_", ""), cv2.aruco.ArucoDetector(d, params)))
        t0 = time.perf_counter()
        main_name = self.config["aruco"]["dictionary"].replace("DICT_", "")
        found, taken = [], []
        detectors = [d for d in self._scan_detectors if families is None or d[0] in families]
        for name, det in detectors:
            corners, ids, _ = det.detectMarkers(gray)
            if ids is None:
                continue
            for marker_id, c in zip(ids.ravel().tolist(), corners):
                c = c.reshape(4, 2)
                centre = c.mean(axis=0)
                # Same square found by a bigger family (4X4_1000 vs 4X4_50...): keep the first.
                if any(((centre - t) ** 2).sum() < 25.0 for t in taken):
                    continue
                if name.startswith(main_name.split("_")[0]):
                    taken.append(centre)
                    continue  # pad family: every ID is already outlined by the main pipeline
                taken.append(centre)
                found.append((f"{name} #{marker_id}", c))
        # One missed scan (blur, motion) should not make the outline blink: a marker stays
        # SCAN_HOLD_S after it was last seen (its last_seen time goes with it -> age_s).
        now = time.monotonic()
        seen = self._scan_seen
        for label, c in found:
            seen[label] = (c, now)
        for label in [k for k, (_c, t) in seen.items() if now - t > SCAN_HOLD_S]:
            del seen[label]
        self._scan_found = [(label, c, t) for label, (c, t) in seen.items()]
        self.scan_ms = (time.perf_counter() - t0) * 1000.0

    def _run(self):
        failures, last_stamp, retry_delay, next_preview = 0, None, 1.0, 0.0
        while self.is_running:
            if self.cam is None:
                try:
                    self._open()
                    failures, retry_delay, last_stamp = 0, 1.0, None
                except Exception as e:  # noqa: BLE001
                    self.last_error = str(e).splitlines()[0]
                    self.error_count += 1
                    self._set_link(camera=False)
                    print(
                        f"[CAMERA_NODE Error] Cannot open camera: {self.last_error}. "
                        f"Retrying in {retry_delay:.0f} s."
                    )
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

            # Optical Flow -> VisionState (always active while camera runs so Loiter works continuously)
            if self.flow_estimator is not None:
                try:
                    flow_res = self.flow_estimator.update(gray, stamp)
                    if flow_res is not None and hasattr(self.vision_state, "update_optical_flow"):
                        self.vision_state.update_optical_flow(
                            rad_x=flow_res["rad_x"],
                            rad_y=flow_res["rad_y"],
                            dt_s=flow_res["dt_s"],
                            quality=flow_res["quality"],
                            px_dx=flow_res["px_dx"],
                            px_dy=flow_res["px_dy"],
                        )
                except Exception as e:  # noqa: BLE001
                    self.error_count += 1
                    self.last_error = f"flow: {e}"

            # ArUco target -> VisionState
            with self._vision_lock:
                if self.vision_enabled:
                    t0 = time.perf_counter()
                    try:
                        good = self.tracker.update(gray, stamp)
                        if getattr(self.vision_state, "scan_any", False):
                            self._scan_any(gray, stamp)
                        self._publish_target(stamp)
                        self._publish_overlay(good, gray.shape)
                        setattr(self.vision_state, "connected", True)
                        setattr(self.vision_state, "current_rate_hz", round(self.fps, 1))
                    except Exception as e:  # noqa: BLE001
                        self.error_count += 1
                        self.last_error = f"vision: {e}"
                    ms = (time.perf_counter() - t0) * 1000.0
                    self.proc_ms = ms if self.proc_ms == 0 else 0.9 * self.proc_ms + 0.1 * ms

            # Video preview -> CameraState (rate-limited, sized for one UDP packet)
            if stamp >= next_preview:
                next_preview = stamp + self.preview_period
                self.camera_state.update_frame(self._encode_preview(gray), fps=self.fps)

        self._close_camera()