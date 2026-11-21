# file: src/interfaces/MAVLink/tx/sensor_feed.py

import math
import threading
import time
from collections import deque
from typing import Any, Optional
from pymavlink import mavutil


class MAVLinkSensorFeed:
    """20 Hz MAVLink transmitter feeding the rangefinder and optical flow to ArduPilot EKF3.

    Sends:
      - DISTANCE_SENSOR  (VL53L1X, pointing down)          -> RNGFND1_TYPE = 10 (MAVLink)
      - OPTICAL_FLOW     (msg 100, flow_rate_x/y in rad/s) -> FLOW_TYPE = 5 (MAVLink)

    ArduPilot's MAVLink flow driver (AP_OpticalFlow_MAV) only decodes OPTICAL_FLOW
    (msg 100). OPTICAL_FLOW_RAD (msg 106) is silently ignored by ArduPilot, so it
    is not sent. The flow is RAW (it includes the vehicle's own rotation):
    ArduPilot removes rotation itself using its own gyro (OF.bodyX/Y).

    ExternalNav (VISION_POSITION_ESTIMATE) is OFF by default: a position made by
    summing optical flow drifts without bound and does not remove tilt, yet the
    EKF would treat it as absolute. Let EKF3 use the flow as velocity
    (EK3_SRC1_VELXY = 5) and work out the relative position itself.
    """

    MIN_FLOW_HEIGHT_M = 0.15
    # Final sign multipliers on the sent rates. Keep at 1.0: set the camera -> body
    # mapping only in camera_driver/config.yaml (optical_flow: swap_xy / invert_x /
    # invert_y), using the 'flowtest' rotation test to choose the values.
    FLOW_SIGN_X = 1.0
    FLOW_SIGN_Y = 1.0
    # Legacy ExternalNav position integrated from flow. Not recommended (see above).
    SEND_EXTERNAL_NAV = False

    def __init__(self, rate_hz: float = 20.0):
        self.rate_hz = max(5.0, float(rate_hz))
        self.master: Optional[Any] = None
        self.distance_state: Optional[Any] = None
        self.vision_state: Optional[Any] = None
        self.vehicle_state: Optional[Any] = None

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._tx_lock = threading.Lock()
        self._boot_time = time.monotonic()
        self._last_lidar_us: int = 0
        self._loop_count: int = 0

        # Bench odometry: flow summed into a position, ONLY for the in-hand checks
        # (print_axis_check / flowcheck). Not sent unless SEND_EXTERNAL_NAV.
        self._pos_x_m: float = 0.0
        self._pos_y_m: float = 0.0
        self._was_armed: bool = False

        self.lidar_packets_sent: int = 0
        self.flow_packets_sent: int = 0
        self.vision_packets_sent: int = 0
        self.actual_rate_hz: float = 0.0

        # Last flow packet (for diagnostics) and a short history for 'flowtest'.
        self.last_flow: dict = {}
        # (time.time(), rate_x, rate_y, quality, height_m or None, gated)
        self.flow_history: deque = deque(maxlen=1200)   # ~60 s at 20 Hz

    def set_master(self, master: Any):
        with self._tx_lock:
            self.master = master

    def bind_states(self, distance_state: Any = None, vision_state: Any = None, vehicle_state: Any = None):
        if distance_state is not None:
            self.distance_state = distance_state
        if vision_state is not None:
            self.vision_state = vision_state
        if vehicle_state is not None:
            self.vehicle_state = vehicle_state

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._feed_loop, name="MAVLinkSensorFeed", daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.actual_rate_hz = 0.0

    def reset_bench_odometry(self):
        self._pos_x_m = 0.0
        self._pos_y_m = 0.0

    # ------------------------------------------------------------- inputs
    def _is_vehicle_armed(self, master: Any) -> bool:
        try:
            if self.vehicle_state is not None and hasattr(self.vehicle_state, "is_armed"):
                return bool(self.vehicle_state.is_armed())
            if hasattr(master, "motors_armed") and callable(master.motors_armed):
                return bool(master.motors_armed())
        except Exception:
            pass
        return False

    def _get_current_yaw_rad(self, master: Any) -> float:
        """Reads live ATTITUDE yaw from VehicleState (preferred) or pymavlink cache."""
        try:
            if self.vehicle_state is not None and hasattr(self.vehicle_state, "get_packet"):
                att_pkt = self.vehicle_state.get_packet("ATTITUDE")
                if att_pkt and "yaw" in att_pkt:
                    return float(att_pkt["yaw"])
            att = master.messages.get("ATTITUDE") if hasattr(master, "messages") else None
            if att is not None and hasattr(att, "yaw"):
                return float(att.yaw)
        except Exception:
            pass
        return 0.0

    def _read_valid_lidar_m(self) -> Optional[float]:
        st = self.distance_state
        if st is None:
            return None
        if not getattr(st, "connected", False):
            return None
        if getattr(st, "sensor_status", "OK") != "OK":
            return None

        dist_m = float(getattr(st, "distance_m", 0.0))
        min_m = float(getattr(st, "min_distance_m", 0.04))
        max_m = float(getattr(st, "max_distance_m", 4.0))
        if min_m <= dist_m <= max_m:
            return dist_m
        return None

    def _take_flow(self) -> Optional[dict]:
        """All camera flow since the last tick, as rates in rad/s (sign-aligned)."""
        vs = self.vision_state
        if vs is None:
            return None
        if hasattr(vs, "take_optical_flow"):
            acc = vs.take_optical_flow()
        else:  # older VisionState: latest sample only
            acc = vs.get_optical_flow() if hasattr(vs, "get_optical_flow") else None
            if acc is not None:
                if int(acc.get("seq", -1)) == getattr(self, "_last_flow_seq", -1):
                    return None
                self._last_flow_seq = int(acc.get("seq", -1))
        if not acc:
            return None
        if time.time() - float(acc.get("ts", 0.0)) > 0.25:
            return None  # camera stalled: send nothing, ArduPilot times out after 0.5 s
        dt_s = float(acc.get("dt_s", 0.0))
        if dt_s <= 1e-4:
            return None
        return {
            "rate_x": self.FLOW_SIGN_X * float(acc.get("rad_x", 0.0)) / dt_s,
            "rate_y": self.FLOW_SIGN_Y * float(acc.get("rad_y", 0.0)) / dt_s,
            "rad_x": self.FLOW_SIGN_X * float(acc.get("rad_x", 0.0)),
            "rad_y": self.FLOW_SIGN_Y * float(acc.get("rad_y", 0.0)),
            "dt_s": dt_s,
            "quality": max(0, min(255, int(acc.get("quality", 0)))),
            "samples": int(acc.get("samples", 1)),
        }

    # ------------------------------------------------------------ bench odometry
    def _integrate_odometry(self, flow: dict, ground_dist_m: Optional[float], yaw_rad: float):
        """Flow summed into a local position. NOT rotation compensated: only valid when
        the vehicle is moved LEVEL (in-hand slide test). Never fused by default."""
        if ground_dist_m is None or ground_dist_m < self.MIN_FLOW_HEIGHT_M:
            return
        if int(flow.get("quality", 0)) < 50:
            return
        # ArduPilot convention: moving forward gives +flow Y, moving right gives -flow X.
        dx_body = flow["rad_y"] * ground_dist_m
        dy_body = -flow["rad_x"] * ground_dist_m
        cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
        self._pos_x_m += dx_body * cos_y - dy_body * sin_y
        self._pos_y_m += dx_body * sin_y + dy_body * cos_y

    # ------------------------------------------------------------ senders
    def _send_distance_sensor(self, master: Any, dist_m: float):
        st = self.distance_state
        min_cm = int(round(float(getattr(st, "min_distance_m", 0.04)) * 100.0))
        max_cm = int(round(float(getattr(st, "max_distance_m", 4.0)) * 100.0))
        cur_cm = max(min_cm, min(max_cm, int(round(dist_m * 100.0))))
        now_mono = time.monotonic()
        time_boot_ms = int((now_mono - self._boot_time) * 1000.0) & 0xFFFFFFFF
        self._last_lidar_us = int((now_mono - self._boot_time) * 1_000_000)

        with self._tx_lock:
            master.mav.distance_sensor_send(
                time_boot_ms,
                min_cm,
                max_cm,
                cur_cm,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                0,
                mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
                255,
            )
        self.lidar_packets_sent += 1

    def _send_optical_flow(self, master: Any, flow: dict, ground_dist_m: Optional[float]):
        """OPTICAL_FLOW (msg 100) with flow_rate_x/y in rad/s, as ArduPilot expects."""
        time_usec = int((time.monotonic() - self._boot_time) * 1_000_000)
        airborne = ground_dist_m is not None and ground_dist_m >= self.MIN_FLOW_HEIGHT_M
        if airborne:
            rate_x, rate_y, quality = flow["rate_x"], flow["rate_y"], flow["quality"]
        else:
            # Too low (or no rangefinder): report "no usable flow"
            rate_x, rate_y, quality = 0.0, 0.0, 0
        ground_distance = float(ground_dist_m) if ground_dist_m is not None else -1.0

        with self._tx_lock:
            master.mav.optical_flow_send(
                time_usec,
                0,               # sensor_id
                0,               # flow_x (dpix, unused: ArduPilot uses flow_rate_x/y)
                0,               # flow_y
                0.0,             # flow_comp_m_x
                0.0,             # flow_comp_m_y
                quality,
                ground_distance, # ignored by ArduPilot (it uses the rangefinder)
                float(rate_x),   # flow_rate_x, rad/s
                float(rate_y),   # flow_rate_y, rad/s
            )
        self.flow_packets_sent += 1
        self.last_flow = {
            "t": time.time(), "rate_x": rate_x, "rate_y": rate_y, "quality": quality,
            "raw_rate_x": flow["rate_x"], "raw_rate_y": flow["rate_y"],
            "raw_quality": flow["quality"], "samples": flow["samples"],
            "height_m": ground_dist_m, "gated": not airborne,
        }
        self.flow_history.append((time.time(), flow["rate_x"], flow["rate_y"],
                                  flow["quality"], ground_dist_m, not airborne))

    def _send_vision_position_estimate(self, master: Any, yaw_rad: float = 0.0):
        time_usec = int((time.monotonic() - self._boot_time) * 1_000_000)
        with self._tx_lock:
            master.mav.vision_position_estimate_send(
                time_usec,
                float(self._pos_x_m),
                float(self._pos_y_m),
                0.0,
                0.0,
                0.0,
                float(yaw_rad),
            )
        self.vision_packets_sent += 1

    def print_axis_check(self):
        """In-hand level slide check (>15 cm): prints the flow and the bench position."""
        lidar_m = self._read_valid_lidar_m()
        f = self.last_flow or {}
        print(
            f"[SensorFeed CHECK] lidar={lidar_m if lidar_m is not None else -1:.2f}m "
            f"qual={f.get('raw_quality', 0)} rate=({f.get('raw_rate_x', 0.0):+.3f}, "
            f"{f.get('raw_rate_y', 0.0):+.3f}) rad/s gated={f.get('gated', True)} "
            f"bench_pos=({self._pos_x_m:+.2f} N, {self._pos_y_m:+.2f} E) m",
            flush=True,
        )

    # ------------------------------------------------------------ loop
    def _feed_loop(self):
        period = 1.0 / self.rate_hz
        last_tick = time.monotonic()

        while self._running:
            loop_start = time.monotonic()
            master = self.master
            self._loop_count += 1

            if master is not None:
                try:
                    armed = self._is_vehicle_armed(master)
                    if armed and not self._was_armed:
                        self.reset_bench_odometry()
                    self._was_armed = armed

                    yaw_rad = self._get_current_yaw_rad(master)
                    lidar_m = self._read_valid_lidar_m()

                    if lidar_m is not None:
                        self._send_distance_sensor(master, lidar_m)

                    flow = self._take_flow()
                    if flow is not None:
                        self._integrate_odometry(flow, lidar_m, yaw_rad)
                        self._send_optical_flow(master, flow, lidar_m)

                    if self.SEND_EXTERNAL_NAV and self._loop_count % 2 == 0:
                        self._send_vision_position_estimate(master, yaw_rad)

                except Exception as e:
                    if not getattr(self, "_err_logged", False):
                        print(f"[SensorFeed ERR] {type(e).__name__}: {e}", flush=True)
                        self._err_logged = True

            if not getattr(self, "_diag_logged", False) and (time.monotonic() - self._boot_time) > 3.0:
                print(
                    f"[SensorFeed DIAG] lidar_tx={self.lidar_packets_sent} "
                    f"flow_tx={self.flow_packets_sent} (OPTICAL_FLOW msg 100) "
                    f"extnav={'ON' if self.SEND_EXTERNAL_NAV else 'OFF'} last_flow={self.last_flow}",
                    flush=True,
                )
                self._diag_logged = True

            now = time.monotonic()
            dt = now - last_tick
            if dt > 0.0:
                inst_hz = 1.0 / dt
                self.actual_rate_hz = inst_hz if self.actual_rate_hz == 0.0 else (0.9 * self.actual_rate_hz + 0.1 * inst_hz)
            last_tick = now

            sleep_time = period - (time.monotonic() - loop_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)
