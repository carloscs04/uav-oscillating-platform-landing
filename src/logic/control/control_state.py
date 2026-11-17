# File: src/logic/control/control_state.py

import threading
from typing import Any, Dict, Optional


class ControlState:
    """Thread-safe state buffer for 3-Axis Flight Control telemetry and health status."""

    def __init__(self):
        self._lock = threading.Lock()
        self.state = self  # Compatibility with ControlTransmitter (hasattr(node, "state"))

        # Flight phase & analog actuator outputs
        self.phase: str = "IDLE"
        self.active_guidance: bool = False
        self.mission_mode: bool = False
        self.sensor_source: str = "NONE"

        # Analog thrust & PID breakdown
        self.thrust: float = 0.500
        self.hover_trim: float = 0.500
        self.p_term: float = 0.0
        self.i_term: float = 0.0
        self.d_term: float = 0.0

        # Vertical (Z) kinematics & reference
        self.z_est_m: float = 0.0
        self.z_ref_m: float = 1.5
        self.z_error_m: float = 0.0
        self.vz_est_mps: float = 0.0
        self.vz_ref_mps: float = 0.0
        self.lidar_m: Optional[float] = None
        self.imu_az_mps2: float = 0.0

        # Horizontal (X, Y) & attitude setpoints
        self.x_est_m: float = 0.0
        self.y_est_m: float = 0.0
        self.x_ref_m: float = 0.0
        self.y_ref_m: float = 0.0
        self.roll_deg: float = 0.0
        self.pitch_deg: float = 0.0
        self.yaw_deg: float = 0.0

        # Human-readable motion directions for Ground Station Inspect Data
        self.z_direction: str = "IDLE"    # UP | DOWN | HOLD | IDLE
        self.xy_direction: str = "CENTER" # CENTER | LEFT | RIGHT | FORWARD | BACKWARD

    def update_control_telemetry(
        self,
        *,
        phase: str,
        active_guidance: bool,
        mission_mode: bool,
        sensor_source: str,
        thrust: float,
        hover_trim: float,
        p_term: float,
        i_term: float,
        d_term: float,
        z_est_m: float,
        z_ref_m: float,
        vz_est_mps: float,
        vz_ref_mps: float,
        lidar_m: Optional[float],
        imu_az_mps2: float,
        x_est_m: float = 0.0,
        y_est_m: float = 0.0,
        x_ref_m: float = 0.0,
        y_ref_m: float = 0.0,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        yaw_deg: float = 0.0,
    ):
        with self._lock:
            self.phase = phase
            self.active_guidance = active_guidance
            self.mission_mode = mission_mode
            self.sensor_source = sensor_source

            self.thrust = round(float(thrust), 4)
            self.hover_trim = round(float(hover_trim), 4)
            self.p_term = round(float(p_term), 4)
            self.i_term = round(float(i_term), 4)
            self.d_term = round(float(d_term), 4)

            self.z_est_m = round(float(z_est_m), 3)
            self.z_ref_m = round(float(z_ref_m), 3)
            self.z_error_m = round(float(z_ref_m - z_est_m), 3)
            self.vz_est_mps = round(float(vz_est_mps), 3)
            self.vz_ref_mps = round(float(vz_ref_mps), 3)
            self.lidar_m = round(float(lidar_m), 3) if lidar_m is not None else None
            self.imu_az_mps2 = round(float(imu_az_mps2), 3)

            self.x_est_m = round(float(x_est_m), 3)
            self.y_est_m = round(float(y_est_m), 3)
            self.x_ref_m = round(float(x_ref_m), 3)
            self.y_ref_m = round(float(y_ref_m), 3)
            self.roll_deg = round(float(roll_deg), 2)
            self.pitch_deg = round(float(pitch_deg), 2)
            self.yaw_deg = round(float(yaw_deg), 2)

            # Vertical action classification
            if not active_guidance or phase == "IDLE":
                self.z_direction = "IDLE"
            elif vz_ref_mps > 0.06 or (thrust - hover_trim) > 0.018:
                self.z_direction = "UP"
            elif vz_ref_mps < -0.06 or (thrust - hover_trim) < -0.018:
                self.z_direction = "DOWN"
            else:
                self.z_direction = "HOLD"

            # Horizontal action classification (roll > 0 = RIGHT, pitch < 0 = FORWARD)
            horiz = []
            if pitch_deg < -1.0 or (y_ref_m - y_est_m) > 0.05:
                horiz.append("FORWARD")
            elif pitch_deg > 1.0 or (y_ref_m - y_est_m) < -0.05:
                horiz.append("BACKWARD")

            if roll_deg > 1.0 or (x_ref_m - x_est_m) > 0.05:
                horiz.append("RIGHT")
            elif roll_deg < -1.0 or (x_ref_m - x_est_m) < -0.05:
                horiz.append("LEFT")

            self.xy_direction = "_".join(horiz) if horiz else "CENTER"

    def export_telemetry_data(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "control_status": {
                    "phase": self.phase,
                    "active": self.active_guidance,
                    "z_direction": self.z_direction,
                    "xy_direction": self.xy_direction,
                    "thrust": self.thrust,
                    "hover_trim": self.hover_trim,
                    "sensor_source": self.sensor_source,
                },
                "altitude_loop": {
                    "z_est_m": self.z_est_m,
                    "z_ref_m": self.z_ref_m,
                    "z_error_m": self.z_error_m,
                    "vz_est_mps": self.vz_est_mps,
                    "vz_ref_mps": self.vz_ref_mps,
                    "lidar_m": self.lidar_m,
                    "imu_az_mps2": self.imu_az_mps2,
                    "p_term": self.p_term,
                    "i_term": self.i_term,
                    "d_term": self.d_term,
                },
                "trajectory": {
                    "x": self.x_est_m,
                    "y": self.y_est_m,
                    "z": self.z_est_m,
                    "ref_x": self.x_ref_m,
                    "ref_y": self.y_ref_m,
                    "ref_z": self.z_ref_m,
                    "roll_deg": self.roll_deg,
                    "pitch_deg": self.pitch_deg,
                    "yaw_deg": self.yaw_deg,
                },
            }

    def export_node_health(self, rate_hz: float, is_connected: bool) -> Dict[str, Any]:
        if not is_connected:
            return {
                "status": "OFFLINE",
                "rate_hz": 0.0,
                "battery_pct": -1,
                "voltage_v": 0.0,
            }
        return {
            "status": "HEALTHY",
            "rate_hz": round(rate_hz, 1),
            "battery_pct": -1,
            "voltage_v": 0.0,
        }