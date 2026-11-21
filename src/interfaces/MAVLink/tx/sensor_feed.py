# file: src/interfaces/MAVLink/tx/sensor_feed.py

import math
import threading
import time
from typing import Any, Optional
from pymavlink import mavutil


class MAVLinkSensorFeed:
    """Dedicated 20Hz MAVLink transmitter injecting LiDAR, Optical Flow, and ExternalNav into Pixhawk EKF3."""

    MIN_FLOW_HEIGHT_M = 0.15
    # Sign multipliers aligning camera optical flow axes with ArduPilot body NED frame:
    # +rad_y -> forward (+X body), -rad_x -> right (+Y body)
    FLOW_SIGN_X = 1.0
    FLOW_SIGN_Y = 1.0

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
        self._last_flow_seq: int = -1
        self._last_lidar_us: int = 0
        self._loop_count: int = 0

        # Integrated visual odometry state (NED local frame in meters)
        self._pos_x_m: float = 0.0
        self._pos_y_m: float = 0.0
        self._was_armed: bool = False

        self.lidar_packets_sent: int = 0
        self.flow_packets_sent: int = 0
        self.vision_packets_sent: int = 0
        self.actual_rate_hz: float = 0.0

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

    def _aligned_flow_rads(self, flow: dict) -> tuple[float, float]:
        rad_x = self.FLOW_SIGN_X * float(flow.get("rad_x", 0.0))
        rad_y = self.FLOW_SIGN_Y * float(flow.get("rad_y", 0.0))
        return rad_x, rad_y

    def _integrate_odometry(self, flow: dict, ground_dist_m: Optional[float], yaw_rad: float):
        """Integrates optical flow angular displacement into local NED X/Y meters for ExternalNav."""
        if ground_dist_m is None or ground_dist_m < self.MIN_FLOW_HEIGHT_M:
            # Keep origin anchored at (0, 0) while sitting on the ground
            self._pos_x_m = 0.0
            self._pos_y_m = 0.0
            return

        quality = int(flow.get("quality", 0))
        if quality < 50:
            return

        rad_x, rad_y = self._aligned_flow_rads(flow)

        # ArduPilot optical_flow_rad convention:
        # +rad_y -> forward (+X body), -rad_x -> right (+Y body)
        dx_body = rad_y * ground_dist_m
        dy_body = -rad_x * ground_dist_m

        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        self._pos_x_m += dx_body * cos_y - dy_body * sin_y
        self._pos_y_m += dx_body * sin_y + dy_body * cos_y

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

    def _send_optical_flow_rad(self, master: Any, flow: dict, ground_dist_m: Optional[float]):
        now_mono = time.monotonic()
        time_usec = int((now_mono - self._boot_time) * 1_000_000)
        dt_s = max(1e-3, min(0.5, float(flow.get("dt_s", 0.033))))
        integration_time_us = int(round(dt_s * 1_000_000))

        # Only report valid optical flow once airborne (>= 15 cm)
        if ground_dist_m is not None and ground_dist_m >= self.MIN_FLOW_HEIGHT_M:
            rad_x, rad_y = self._aligned_flow_rads(flow)
            quality = max(0, min(255, int(flow.get("quality", 0))))
            distance = float(ground_dist_m)
            time_delta_distance_us = max(0, time_usec - self._last_lidar_us) if self._last_lidar_us > 0 else 0
        else:
            rad_x = 0.0
            rad_y = 0.0
            quality = 0
            distance = float(ground_dist_m) if ground_dist_m is not None else -1.0
            time_delta_distance_us = 0

        with self._tx_lock:
            master.mav.optical_flow_rad_send(
                time_usec,
                0,
                integration_time_us,
                rad_x,
                rad_y,
                0.0,
                0.0,
                0.0,
                0,
                quality,
                time_delta_distance_us,
                distance,
            )
        self.flow_packets_sent += 1

    def _send_vision_position_estimate(self, master: Any, yaw_rad: float = 0.0):
        now_mono = time.monotonic()
        time_usec = int((now_mono - self._boot_time) * 1_000_000)

        with self._tx_lock:
            master.mav.vision_position_estimate_send(
                time_usec,
                float(self._pos_x_m),  # x (m, North)
                float(self._pos_y_m),  # y (m, East)
                0.0,                   # z (m, Down - zeroed so RangeFinder owns altitude)
                0.0,                   # roll (rad)
                0.0,                   # pitch (rad)
                float(yaw_rad),        # yaw (rad)
            )
        self.vision_packets_sent += 1

    def print_axis_check(self):
        """Helper to verify body-frame flow direction in hand at >15cm without arming."""
        lidar_m = self._read_valid_lidar_m()
        if self.vision_state is None or not hasattr(self.vision_state, "get_optical_flow"):
            print("[SensorFeed CHECK] vision_state not bound", flush=True)
            return
        flow = self.vision_state.get_optical_flow()
        rad_x, rad_y = self._aligned_flow_rads(flow)
        dist = lidar_m if lidar_m is not None else 0.0
        dx = rad_y * dist
        dy = -rad_x * dist
        print(
            f"[SensorFeed CHECK] lidar={dist:.2f}m qual={flow.get('quality', 0)} "
            f"aligned_rad=({rad_x:+.4f}, {rad_y:+.4f}) -> body_step=({dx:+.4f}m Fwd, {dy:+.4f}m Right) "
            f"ned_pos=({self._pos_x_m:+.2f}, {self._pos_y_m:+.2f})",
            flush=True,
        )

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
                        self._pos_x_m = 0.0
                        self._pos_y_m = 0.0
                    self._was_armed = armed

                    yaw_rad = self._get_current_yaw_rad(master)
                    lidar_m = self._read_valid_lidar_m()

                    if lidar_m is not None:
                        self._send_distance_sensor(master, lidar_m)

                    if self.vision_state is not None and hasattr(self.vision_state, "get_optical_flow"):
                        flow = self.vision_state.get_optical_flow()
                        seq = int(flow.get("seq", -1))
                        if seq > self._last_flow_seq and (time.time() - float(flow.get("ts", 0.0))) < 0.25:
                            self._last_flow_seq = seq
                            self._integrate_odometry(flow, lidar_m, yaw_rad)
                            self._send_optical_flow_rad(master, flow, lidar_m)

                    # Keep ExternalNav alive at 10Hz for EK3_SRC1_POSXY & EK3_SRC1_YAW = ExternalNav
                    if self._loop_count % 2 == 0:
                        self._send_vision_position_estimate(master, yaw_rad)

                except Exception as e:
                    if not getattr(self, "_err_logged", False):
                        print(f"[SensorFeed ERR] {type(e).__name__}: {e}", flush=True)
                        self._err_logged = True

            if not getattr(self, "_diag_logged", False) and (time.monotonic() - self._boot_time) > 3.0:
                flow_snap = self.vision_state.get_optical_flow() if (self.vision_state and hasattr(self.vision_state, "get_optical_flow")) else None
                print(
                    f"[SensorFeed DIAG] lidar_tx={self.lidar_packets_sent} "
                    f"flow_tx={self.flow_packets_sent} vision_tx={self.vision_packets_sent} "
                    f"pos=({self._pos_x_m:.2f}, {self._pos_y_m:.2f}) flow={flow_snap}",
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