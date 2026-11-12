# File: src/logic/vision/master_vision.py

import os
import threading
import time
from typing import Any, Optional

import yaml

from src.logic.vision.aruco_detector import ArucoDetector
from src.logic.vision.pose import CameraModel, marker_attitude_deg
from src.logic.vision.tracker import TargetTracker
from src.logic.vision.vision_state import VisionState


def load_vision_config(config_path: Optional[str] = None) -> dict:
    """Read src/logic/vision/config.yaml (aruco + tracking + settings)."""
    if config_path is None:
        config_path = os.path.join(os.path.dirname(__file__), "config.yaml")
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def marker_sizes_m(aruco_cfg: dict) -> dict:
    """{marker_id: black-square side in metres} from the aruco config section."""
    ids = aruco_cfg.get("ids") or list(range(1000))
    sizes = {int(i): float(aruco_cfg["marker_mm"]) / 1000.0 for i in ids}
    sizes.update({int(k): float(v) / 1000.0 for k, v in (aruco_cfg.get("sizes_mm") or {}).items()})
    return sizes


class VisionManager:
    """Main Facade API & Lifecycle Manager for ArUco target perception.

    Consumes frames from a CameraManager and owns VisionState (aruco_pose +
    tracking status, sent on port 5006).

    aruco_pose: x, y, z, distance_m in METRES, camera frame (x right, y down,
    z out of the lens); roll/pitch/yaw in degrees, (0, 0, 0) = marker facing
    the camera squarely. tracking_state: LOCKED (marker in this frame),
    HOLDING (lost for less than tracking.hold_s), SEARCHING (no marker).

        vision = VisionManager(camera_manager)
        vision.start()
        vision.export_telemetry_package()      # VisionState packets + node_health
        vision.stop()
    """

    def __init__(self, camera_manager: Any, config_path: Optional[str] = None):
        self.camera = camera_manager
        self.config_path = config_path
        self.config = self._load_config()
        self.frame_timeout = float(self.config.get("settings", {}).get("frame_timeout", 1.0))

        self.tracker: Optional[TargetTracker] = None
        self.camera_model: Optional[CameraModel] = None
        self._camera_generation = -1

        self.connected: bool = False
        self.running: bool = False
        self.current_rate_hz: float = 0.0
        self.proc_ms: float = 0.0
        self._worker_thread: Optional[threading.Thread] = None

        self.state = VisionState()
        self.state.state = self.state          # tx/*.py compatibility (node.state)
        self._sync_state_link()

        self.error_count = 0
        self.last_error = ""

    def _load_config(self) -> dict:
        return load_vision_config(self.config_path)

    # ------------------------------------------------------------- lifecycle
    def start(self, timeout: float = 3.0) -> bool:
        """Starts the background perception loop (needs the camera running)."""
        if self.running:
            return self.connected

        self.running = True
        self._worker_thread = threading.Thread(
            target=self._perception_loop, name="VisionManager", daemon=True)
        self._worker_thread.start()

        start_wait = time.time()
        while time.time() - start_wait < timeout:
            if self.connected:
                return True
            time.sleep(0.1)
        return self.connected

    def stop(self):
        """Halts the perception loop and publishes 'no target'."""
        self.running = False
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=3.0)
            self._worker_thread = None
        self.connected = False
        self.current_rate_hz = 0.0
        self._sync_state_link()
        self.state.update_aruco_pose(detected=False, tracking_state="SEARCHING")
        print("[VISION] Node stopped cleanly.")

    def is_healthy(self) -> bool:
        return self.connected

    def export_telemetry_package(self) -> dict:
        """VisionState packets (aruco_pose, status) + node_health."""
        package = self.state.export_telemetry_data()
        package["node_health"] = self.state.export_node_health(self.current_rate_hz, self.connected)
        return package

    # ------------------------------------------------------------ internals
    def _sync_state_link(self):
        """Mirror link status on VisionState (Bridge_WIFI tx/vision.py reads it there)."""
        setattr(self.state, "connected", self.connected)
        setattr(self.state, "current_rate_hz", round(self.current_rate_hz, 1))

    def _build_tracker(self):
        """(Re)build the pipeline for the camera that is open now (its calibration)."""
        calib = self.camera.calibration
        if calib is not None:
            model = CameraModel.from_calibration(calib, self.camera.calib_key)
        elif self.camera.nominal_focal_px:
            model = CameraModel.nominal(self.camera.resolution, float(self.camera.nominal_focal_px),
                                        self.camera.calib_key)
            print(f"[VISION Warning] {self.camera.calib_key} not calibrated: nominal focal "
                  f"{self.camera.nominal_focal_px} px, distances are approximate.")
        else:
            raise RuntimeError(f"no calibration and no nominal_focal_px for {self.camera.calib_key}")

        aruco, tracking = self.config["aruco"], self.config.get("tracking", {})
        self.tracker = TargetTracker(
            detector=ArucoDetector(aruco["dictionary"], aruco.get("ids")),
            camera_model=model, sizes_m=marker_sizes_m(aruco),
            max_reproj_err_px=float(aruco.get("max_reproj_err_px", 2.0)),
            primary_ids=tracking.get("primary_ids") or aruco.get("ids"),
            smoothing=float(tracking.get("smoothing", 0.6)),
            max_jump_m=float(tracking.get("max_jump_m", 0.30)),
            hold_s=float(tracking.get("hold_s", 0.5)))
        self.camera_model = model
        self._camera_generation = self.camera.camera_generation
        print(f"[VISION] Tracking {aruco['dictionary']} ids={aruco.get('ids')} "
              f"size {aruco['marker_mm']:g} mm with {self.camera.calib_key} "
              f"(calibrated={model.calibrated})")

    def _publish(self, stamp: float):
        t = self.tracker.target(stamp)
        if not t.get("detected"):
            self.state.update_aruco_pose(detected=False, tracking_state="SEARCHING")
            return
        roll, pitch, yaw = marker_attitude_deg(t["rvec"])
        self.state.update_aruco_pose(
            detected=True, target_id=int(t["id"]),
            x=t["x"], y=t["y"], z=t["z"],
            roll=roll, pitch=pitch, yaw=yaw,
            tracking_state="LOCKED" if t["in_view"] else "HOLDING")

    def _perception_loop(self):
        last_seq, last_stamp, waiting_logged = 0, None, False
        while self.running:
            item = self.camera.wait_for_frame(last_seq, timeout=self.frame_timeout)
            if item is None:
                # WATCHDOG: no frames (camera stopped / reconnecting)
                if self.connected or not waiting_logged:
                    print("[VISION] Waiting for camera frames...")
                    waiting_logged = True
                self.connected, self.current_rate_hz = False, 0.0
                self._sync_state_link()
                self.state.update_aruco_pose(detected=False, tracking_state="SEARCHING")
                continue
            waiting_logged = False
            last_seq, gray, stamp = item

            try:
                if self.tracker is None or self._camera_generation != self.camera.camera_generation:
                    self._build_tracker()
                t0 = time.perf_counter()
                self.tracker.update(gray, stamp)
                self._publish(stamp)
                ms = (time.perf_counter() - t0) * 1000.0
                self.proc_ms = ms if self.proc_ms == 0 else 0.9 * self.proc_ms + 0.1 * ms
            except Exception as e:  # one bad frame must not kill the node
                self.error_count += 1
                self.last_error = str(e)
                continue

            if last_stamp is not None and stamp > last_stamp:
                inst = 1.0 / (stamp - last_stamp)
                self.current_rate_hz = inst if self.current_rate_hz == 0 else \
                    0.9 * self.current_rate_hz + 0.1 * inst
            last_stamp = stamp
            self.connected = True
            self._sync_state_link()


if __name__ == "__main__":
    # Standalone check:  python3 -m src.logic.vision.master_vision
    from src.inputs.camera_driver.master_camera import CameraManager

    camera = CameraManager()
    vision = VisionManager(camera)
    camera.start()
    vision.start()
    try:
        while True:
            time.sleep(1.0)
            pkg = vision.export_telemetry_package()
            p, st, h = pkg["aruco_pose"], pkg["status"], pkg["node_health"]
            target = (f"ID {p['id']} x{p['x']:+.3f} y{p['y']:+.3f} z{p['z']:.3f} "
                      f"d={p['distance_m']:.3f} m" if p["detected"] else "no marker")
            print(f"[VISION] {h['status']} {h['rate_hz']} Hz {vision.proc_ms:.0f} ms | "
                  f"{st['tracking_state']} | {target}")
    except KeyboardInterrupt:
        pass
    finally:
        vision.stop()
        camera.stop()
