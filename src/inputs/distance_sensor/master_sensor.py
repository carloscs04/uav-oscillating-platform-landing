# File: src/inputs/distance_sensor/master_sensor.py

import os
import threading
import time
from typing import Any, Optional

import yaml

from src.inputs.distance_sensor.sensor_state import DistanceSensorState


class DistanceSensorManager:
    """Main Facade API & Lifecycle Manager for the distance sensor (rangefinder).

    Owns DistanceSensorState (distance_sensor packet, sent on port 5002).

        sensor = DistanceSensorManager()
        sensor.start()                     # background open / auto-reconnect supervisor
        sensor.export_telemetry_package()  # DistanceSensorState packets + node_health
        sensor.stop()

    Sensor: VL53L1X time-of-flight over I2C (config.yaml `type: vl53l1x_i2c`).
    A sample with no valid target (too far, nothing below) keeps the last distance
    and sets status NO_TARGET; only a silent sensor counts as disconnected.
    """

    # Valid range (m) published in DistanceSensorState for each sensor / mode.
    RANGES_M = {("vl53l1x_i2c", "long"): (0.04, 4.0), ("vl53l1x_i2c", "short"): (0.04, 1.3)}

    def __init__(self, config_path: Optional[str] = None):
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "config.yaml")

        self.config_path = config_path
        self.config = self._load_config()
        settings = self.config.get("settings", {})
        self.read_timeout = float(settings.get("read_timeout", 1.0))
        self.period = 1.0 / max(1.0, float(self.config.get("rate_hz", 20)))

        self.device: Optional[Any] = None
        self.connected: bool = False
        self.running: bool = False
        self.current_rate_hz: float = 0.0
        self._supervisor_thread: Optional[threading.Thread] = None

        self.sensor_type = self.config.get("type", "none")
        self.state = DistanceSensorState()
        self.state.state = self.state      # tx/*.py compatibility (node.state)
        mode = self.config.get("vl53l1x", {}).get("distance_mode") \
            if self.sensor_type == "vl53l1x_i2c" else None
        if (self.sensor_type, mode) in self.RANGES_M:
            self.state.min_distance_m, self.state.max_distance_m = \
                self.RANGES_M[(self.sensor_type, mode)]
        self._last_distance_m = 0.0
        self._sync_state_link()
        self.last_error = ""

    def _load_config(self) -> dict:
        with open(self.config_path, "r") as f:
            return yaml.safe_load(f)["distance_sensor"]

    # ------------------------------------------------------------- lifecycle
    def start(self, timeout: float = 3.0) -> bool:
        if self.running:
            return self.connected
        self.running = True
        self._supervisor_thread = threading.Thread(
            target=self._sensor_supervisor, name="DistanceSensorManager", daemon=True)
        self._supervisor_thread.start()

        start_wait = time.time()
        while time.time() - start_wait < timeout:
            if self.connected:
                return True
            time.sleep(0.1)
        return self.connected

    def stop(self):
        self.running = False
        if self._supervisor_thread is not None:
            self._supervisor_thread.join(timeout=3.0)
            self._supervisor_thread = None
        self._cleanup_device()
        print("[DISTANCE] Node stopped cleanly.")

    def is_healthy(self) -> bool:
        return self.connected

    def export_telemetry_package(self) -> dict:
        package = self.state.export_telemetry_data()
        package["node_health"] = self.state.export_node_health(self.current_rate_hz, self.connected)
        return package

    # -------------------------------------------------------------- hardware
    def _open_device(self):
        """Open and start the sensor configured in config.yaml. Returns the driver."""
        bus = int(self.config.get("i2c_bus", 1))
        address = int(self.config.get("i2c_address", 0x29))
        if self.sensor_type == "vl53l1x_i2c":
            from src.inputs.distance_sensor.vl53l1x import VL53L1X
            cfg = self.config.get("vl53l1x", {})
            dev = VL53L1X(bus=bus, address=address,
                          distance_mode=cfg.get("distance_mode", "long"),
                          timing_budget_ms=int(cfg.get("timing_budget_ms", 50)),
                          period_ms=round(self.period * 1000))
            dev.start_ranging()
            return dev
        raise NotImplementedError(f"distance sensor type '{self.sensor_type}' has no driver "
                                  f"(see config.yaml `type`)")

    def _read_distance_m(self) -> Optional[float]:
        """Block for the next sample. Returns metres, or None if the sensor answered
        but saw no valid target. Raises if the sensor does not answer."""
        mm = self.device.read_mm()
        return None if mm is None else mm / 1000.0

    # ------------------------------------------------------------ internals
    def _sync_state_link(self):
        """Mirror link status on DistanceSensorState (Bridge_WIFI tx reads it there)."""
        setattr(self.state, "connected", self.connected)
        setattr(self.state, "current_rate_hz", round(self.current_rate_hz, 1))

    def _cleanup_device(self):
        self.connected = False
        self.current_rate_hz = 0.0
        self._sync_state_link()
        if self.device is not None:
            try:
                close = getattr(self.device, "close", None)
                if callable(close):
                    close()
            except Exception:
                pass
            self.device = None

    def _sensor_supervisor(self):
        attempt_logged, retry_delay = False, 1.0
        last_ok, last_stamp = 0.0, None
        while self.running:
            # 1. AUTO-RECONNECT
            if self.device is None:
                if not attempt_logged:
                    print("[DISTANCE] Connecting...")
                    attempt_logged = True
                try:
                    self.device = self._open_device()
                    self.connected = True
                    attempt_logged, retry_delay, last_ok = False, 1.0, time.monotonic()
                    self._sync_state_link()
                    print("[DISTANCE] Connected (ONLINE)")
                except Exception as e:
                    if str(e) != self.last_error:
                        print(f"[DISTANCE] {e}")
                    self.last_error = str(e)
                    self._cleanup_device()
                    deadline = time.monotonic() + retry_delay
                    while self.running and time.monotonic() < deadline:
                        time.sleep(0.1)
                    retry_delay = min(retry_delay * 2, 10.0)
                    continue

            # 2. READ (the driver paces itself: it waits for the sensor's next sample)
            answered = True
            try:
                dist = self._read_distance_m()
            except Exception as e:
                answered, dist, self.last_error = False, None, str(e)
            now = time.monotonic()
            if answered:
                last_ok = now
                if dist is not None:
                    self._last_distance_m = dist
                    self.state.update_distance(dist, status="OK")
                else:
                    self.state.update_distance(self._last_distance_m, status="NO_TARGET")
                if last_stamp is not None and now > last_stamp:
                    inst = 1.0 / (now - last_stamp)
                    self.current_rate_hz = inst if self.current_rate_hz == 0 else \
                        0.9 * self.current_rate_hz + 0.1 * inst
                last_stamp = now
                self._sync_state_link()

            # 3. WATCHDOG
            if now - last_ok > self.read_timeout:
                print("[DISTANCE] Disconnected (no valid readings). Reconnecting...")
                self._cleanup_device()
                continue

        self._cleanup_device()


if __name__ == "__main__":
    # Standalone check:  python3 -m src.inputs.distance_sensor.master_sensor
    manager = DistanceSensorManager()
    manager.start()
    try:
        while True:
            time.sleep(1.0)
            pkg = manager.export_telemetry_package()
            print(f"[DISTANCE] {pkg['node_health']['status']} {pkg['node_health']['rate_hz']} Hz | "
                  f"{pkg['distance_sensor']}")
    except KeyboardInterrupt:
        pass
    finally:
        manager.stop()
