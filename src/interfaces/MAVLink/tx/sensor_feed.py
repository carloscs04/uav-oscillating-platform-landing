# File: src/interfaces/MAVLink/tx/sensor_feed.py

import threading
import time
from typing import Any, Optional
from pymavlink import mavutil


class MAVLinkSensorFeed:
    """Dedicated 30Hz MAVLink transmitter injecting LiDAR and Optical Flow into Pixhawk EKF3.

    Sends:
      - DISTANCE_SENSOR (downward VL53L1X rangefinder in cm, orientation = PITCH_270 / Down)
      - OPTICAL_FLOW_RAD (integrated angular flow in radians + quality 0..255)
    """

    def __init__(self, rate_hz: float = 30.0):
        self.rate_hz = max(5.0, float(rate_hz))
        self.master: Optional[Any] = None
        self.distance_state: Optional[Any] = None
        self.vision_state: Optional[Any] = None

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._tx_lock = threading.Lock()
        self._boot_time = time.monotonic()
        self._last_flow_seq: int = -1

        # Telemetry counters for diagnostics
        self.lidar_packets_sent: int = 0
        self.flow_packets_sent: int = 0
        self.actual_rate_hz: float = 0.0

    def set_master(self, master: Any):
        """Binds the active pymavlink connection handle."""
        with self._tx_lock:
            self.master = master

    def bind_states(self, distance_state: Any = None, vision_state: Any = None):
        """Binds thread-safe DistanceState and VisionState buffers."""
        self.distance_state = distance_state
        self.vision_state = vision_state

    def start(self):
        """Starts the 30Hz background sensor injection loop."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._feed_loop, name="MAVLinkSensorFeed", daemon=True)
        self._thread.start()

    def stop(self):
        """Stops the background sensor injection loop cleanly."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.actual_rate_hz = 0.0

    def _read_valid_lidar_m(self) -> Optional[float]:
        """Reads current valid downward rangefinder distance in meters."""
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

    def _send_distance_sensor(self, master: Any, dist_m: float):
        """Transmits MAVLink DISTANCE_SENSOR (25 = downward facing)."""
        st = self.distance_state
        min_cm = int(round(float(getattr(st, "min_distance_m", 0.04)) * 100.0))
        max_cm = int(round(float(getattr(st, "max_distance_m", 4.0)) * 100.0))
        cur_cm = max(min_cm, min(max_cm, int(round(dist_m * 100.0))))
        time_boot_ms = int((time.monotonic() - self._boot_time) * 1000.0) & 0xFFFFFFFF

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
        """Transmits MAVLink OPTICAL_FLOW (ID 100) for ArduCopter EKF3 velocity fusion."""
        time_usec = int(time.time() * 1_000_000)
        dt_s = max(1e-3, float(flow.get("dt_s", 0.033)))
        rad_x = float(flow.get("rad_x", 0.0))
        rad_y = float(flow.get("rad_y", 0.0))
        flow_rate_x = rad_x / dt_s
        flow_rate_y = rad_y / dt_s
        flow_x_dpix = max(-32768, min(32767, int(round(float(flow.get("px_dx", 0.0)) * 10.0))))
        flow_y_dpix = max(-32768, min(32767, int(round(float(flow.get("px_dy", 0.0)) * 10.0))))
        quality = max(0, min(255, int(flow.get("quality", 0))))
        distance = float(ground_dist_m) if ground_dist_m is not None else -1.0
        comp_vx = (flow_rate_x * distance) if distance > 0.0 else 0.0
        comp_vy = (flow_rate_y * distance) if distance > 0.0 else 0.0

        with self._tx_lock:
            master.mav.optical_flow_send(
                time_usec,
                0,              # sensor_id
                flow_x_dpix,    # flow_x (deci-pixels)
                flow_y_dpix,    # flow_y (deci-pixels)
                comp_vx,        # flow_comp_m_x (m/s)
                comp_vy,        # flow_comp_m_y (m/s)
                quality,        # quality (0..255)
                distance,       # ground_distance (m)
                flow_rate_x,    # flow_rate_x (rad/s, MAVLink 2)
                flow_rate_y,    # flow_rate_y (rad/s, MAVLink 2)
            )
        self.flow_packets_sent += 1

    def _feed_loop(self):
        period = 1.0 / self.rate_hz
        last_tick = time.monotonic()

        while self._running:
            loop_start = time.monotonic()
            master = self.master

            if master is not None:
                try:
                    lidar_m = self._read_valid_lidar_m()
                    if lidar_m is not None:
                        self._send_distance_sensor(master, lidar_m)

                    if self.vision_state is not None and hasattr(self.vision_state, "get_optical_flow"):
                        flow = self.vision_state.get_optical_flow()
                        seq = int(flow.get("seq", -1))
                        if seq > self._last_flow_seq and (time.time() - float(flow.get("ts", 0.0))) < 0.25:
                            self._last_flow_seq = seq
                            self._send_optical_flow_rad(master, flow, lidar_m)
                except Exception as e:
                    if not getattr(self, "_err_logged", False):
                        print(f"[SensorFeed ERR] {type(e).__name__}: {e}", flush=True)
                        self._err_logged = True

            if not getattr(self, "_diag_logged", False) and (time.monotonic() - self._boot_time) > 3.0:
                flow_snap = self.vision_state.get_optical_flow() if (self.vision_state and hasattr(self.vision_state, "get_optical_flow")) else None
                print(f"[SensorFeed DIAG] lidar_tx={self.lidar_packets_sent} flow_tx={self.flow_packets_sent} flow={flow_snap}", flush=True)
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