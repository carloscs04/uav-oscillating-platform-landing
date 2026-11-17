# File: src/logic/control/master_control.py

import math
import threading
import time
from enum import Enum
from typing import Any, Optional, Tuple
from src.logic.control.control_state import ControlState


class MissionPhase(str, Enum):
    IDLE = "IDLE"
    ARMING = "ARMING"
    CLIMBING = "CLIMBING"
    HOVER = "HOVER"
    ALIGNING = "ALIGNING"
    LANDING = "LANDING"


class ControlManager:
    """Facade API & High-Response Cascaded Flight Controller (50Hz Pixhawk EKF/IMU + VL53L1X)."""

    GRAVITY_MSS = 9.80665

    # Battery-compensated hover feedforward (3S LiPo nominal reference)
    BASE_HOVER_THRUST = 0.500
    NOMINAL_VOLTAGE_V = 11.40
    MIN_HOVER_FF = 0.380
    MAX_HOVER_FF = 0.640

    FALLBACK_CLIMB_THRUST = 0.580
    MIN_THRUST = 0.220
    MAX_THRUST = 0.800

    # Outer loop: Altitude reference (m) -> Desired vertical velocity (m/s)
    POS_KP = 2.40
    POS_DEADBAND_M = 0.012       # +/- 1.2 cm micro-deadband
    MAX_CLIMB_VZ = 1.10          # m/s max upward velocity
    MAX_DESCENT_VZ = 0.85        # m/s max downward velocity

    # Inner fast loop (50 Hz): EKF/IMU velocity & acceleration damping -> Analog Thrust
    VEL_KP = 0.44
    VEL_KI = 0.09
    VEL_KD = 0.06                # Direct velocity-change damping
    ACC_KD = 0.028               # Instantaneous IMU vertical acceleration damping
    VEL_I_MAX = 0.07             # Small trim bound (voltage sag handled by feedforward)
    THRUST_SMOOTH_ALPHA = 0.92   # Near-instant 50Hz analog output

    # Complementary Observer (50Hz Pixhawk EKF/IMU + 30Hz VL53L1X)
    OBS_KZ = 0.42
    OBS_KV = 0.55
    OBS_KB = 0.08

    # Sensor fail-safe thresholds
    LIDAR_WARN_SEC = 1.5
    LIDAR_LAND_SEC = 3.5

    # Reference stabilization criteria to transition status CLIMBING -> HOVER
    ALT_TOLERANCE_M = 0.05
    VZ_STABLE_THRESH = 0.15
    STABLE_HOLD_SEC = 0.40

    def __init__(
        self,
        mavlink_node: Optional[Any] = None,
        vision_node: Optional[Any] = None,
        distance_node: Optional[Any] = None,
        target_hz: float = 50.0,
    ):
        self.mavlink = mavlink_node
        self.vision = vision_node
        self.distance = distance_node

        self.target_hz = target_hz
        self.active_guidance: bool = False
        self.mission_mode: bool = False
        self.phase: MissionPhase = MissionPhase.IDLE
        self.target_altitude_m: float = 1.5
        self.setpoint: Tuple[float, float, float] = (0.0, 0.0, 1.5)

        # Complementary filter & cascaded controller state
        self._est_z: float = 0.0
        self._est_vz: float = 0.0
        self._prev_est_vz: float = 0.0
        self._az_bias: float = 0.0
        self._last_az_corr: float = 0.0
        self._estimator_initialized: bool = False
        self._last_est_time: Optional[float] = None
        self._last_imu_sig: Optional[Tuple[Any, ...]] = None
        self._last_lidar_val: Optional[float] = None
        self._last_valid_lidar_time: float = 0.0
        self._lidar_fault_active: bool = False

        self._vel_integral: float = 0.0
        self._stable_since: Optional[float] = None
        self._current_hover_ff: float = self.BASE_HOVER_THRUST
        self.last_thrust: float = self.BASE_HOVER_THRUST

        # Attitude & XY targets
        self._cmd_roll_deg: float = 0.0
        self._cmd_pitch_deg: float = 0.0
        self._cmd_yaw_deg: float = 0.0
        self._x_est_m: float = 0.0
        self._y_est_m: float = 0.0

        self.state = ControlState()
        self.connected: bool = False
        self.running: bool = False
        self._thread: Optional[threading.Thread] = None

    def bind_subsystems(self, mavlink_node: Any, vision_node: Any = None, distance_node: Any = None):
        self.mavlink = mavlink_node
        self.vision = vision_node
        self.distance = distance_node

    def start(self) -> bool:
        if self.running:
            return self.connected
        self.running = True
        self.connected = True
        setattr(self.state, "connected", True)
        setattr(self.state, "current_rate_hz", self.target_hz)

        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self.active_guidance = False
        self.mission_mode = False
        self.phase = MissionPhase.IDLE
        self.running = False
        self.connected = False
        self._reset_vertical_controller()
        self._publish_telemetry_state(vz_sp=0.0, p_term=0.0, d_term=0.0, cur_lidar=None)
        setattr(self.state, "connected", False)
        setattr(self.state, "current_rate_hz", 0.0)

    def is_healthy(self) -> bool:
        return self.running and self.connected

    def is_airborne(self) -> bool:
        return self.active_guidance and self.phase in {
            MissionPhase.ARMING,
            MissionPhase.CLIMBING,
            MissionPhase.HOVER,
            MissionPhase.ALIGNING,
            MissionPhase.LANDING,
        }

    # =========================================================================
    # BASIC FLIGHT BEHAVIORS (TAKEOFF / HOVER / LAND WITHOUT MISSION ALIGNMENT)
    # =========================================================================

    def start_basic_flight(self, altitude_m: float = 1.5) -> bool:
        if not self.mavlink:
            return False

        self.mission_mode = False
        self.target_altitude_m = float(altitude_m)
        self.setpoint = (0.0, 0.0, self.target_altitude_m)
        self._cmd_roll_deg = 0.0
        self._cmd_pitch_deg = 0.0
        self._stable_since = None
        self.active_guidance = True

        if self.mavlink.state.is_armed() and self.phase in {MissionPhase.CLIMBING, MissionPhase.HOVER, MissionPhase.ALIGNING}:
            self.phase = MissionPhase.CLIMBING
        else:
            self._reset_vertical_controller(initial_alt=self._get_current_altitude_m())
            self.phase = MissionPhase.ARMING
        return True

    def hold_hover(self, force_sim: bool = False) -> bool:
        if not self.mavlink:
            return False
        if not force_sim and not self.mavlink.state.is_armed():
            return False

        cur_alt = self._est_z if self._estimator_initialized else self._get_current_altitude_m()
        if cur_alt is not None and self.phase == MissionPhase.CLIMBING:
            self.target_altitude_m = cur_alt
            self.setpoint = (self.setpoint[0], self.setpoint[1], cur_alt)

        self._cmd_roll_deg = 0.0
        self._cmd_pitch_deg = 0.0
        self._stable_since = None
        self.mission_mode = False
        self.active_guidance = True
        self.phase = MissionPhase.HOVER

        if self.mavlink.is_healthy():
            thrust, _, _ = self._compute_cascaded_thrust(time.monotonic())
            self.mavlink.send_attitude_target(0.0, 0.0, self._cmd_yaw_deg, thrust)
        return True

    def command_land(self) -> bool:
        if not self.mavlink:
            return False

        self.mission_mode = False
        self.active_guidance = False
        self.phase = MissionPhase.IDLE
        self._cmd_roll_deg = 0.0
        self._cmd_pitch_deg = 0.0
        self._reset_vertical_controller()
        self._publish_telemetry_state(vz_sp=0.0, p_term=0.0, d_term=0.0, cur_lidar=self._get_current_altitude_m())
        if self.mavlink.is_healthy():
            return self.mavlink.land()
        return False

    # =========================================================================
    # FULL AUTONOMOUS MISSION (WORKS FROM GROUND OR MID-AIR)
    # =========================================================================

    def start_mission(self, altitude_m: float = 1.5) -> bool:
        if not self.mavlink:
            return False

        self.mission_mode = True
        self.target_altitude_m = float(altitude_m)
        self.setpoint = (0.0, 0.0, self.target_altitude_m)
        self._stable_since = None
        self.active_guidance = True

        if self.mavlink.state.is_armed():
            self.phase = MissionPhase.ALIGNING
        else:
            self._reset_vertical_controller(initial_alt=self._get_current_altitude_m())
            self.phase = MissionPhase.ARMING
        return True

    def abort_mission(self):
        self.mission_mode = False
        self.active_guidance = False
        self.phase = MissionPhase.IDLE
        self._reset_vertical_controller()

    def enable_guidance(self):
        self.active_guidance = True

    def disable_guidance(self):
        self.mission_mode = False
        self.active_guidance = False
        self.phase = MissionPhase.IDLE
        self._reset_vertical_controller()

    def set_target_offset(self, ref_x: float, ref_y: float, ref_z: float):
        old_z = self.target_altitude_m
        self.setpoint = (float(ref_x), float(ref_y), float(ref_z))
        if ref_z > 0.05:
            self.target_altitude_m = float(ref_z)
            if abs(self.target_altitude_m - old_z) > self.ALT_TOLERANCE_M:
                self._stable_since = None
                if self.phase == MissionPhase.HOVER:
                    self.phase = MissionPhase.CLIMBING

    # =========================================================================
    # BATTERY VOLTAGE FEEDFORWARD & SENSOR READERS
    # =========================================================================

    def _get_dynamic_hover_thrust(self) -> float:
        """Adjusts base hover throttle automatically as battery voltage drops."""
        if not self.mavlink or not hasattr(self.mavlink, "state"):
            return self.BASE_HOVER_THRUST

        get_pkt = getattr(self.mavlink.state, "get_packet", None)
        if not callable(get_pkt):
            return self.BASE_HOVER_THRUST

        voltage_v: Optional[float] = None
        sys_st = get_pkt("SYS_STATUS")
        if isinstance(sys_st, dict) and isinstance(sys_st.get("voltage_battery"), (int, float)):
            mv = float(sys_st["voltage_battery"])
            if mv > 1000.0:
                voltage_v = mv / 1000.0

        if voltage_v is None:
            bat_st = get_pkt("BATTERY_STATUS")
            if isinstance(bat_st, dict) and isinstance(bat_st.get("voltages"), (list, tuple)) and bat_st["voltages"]:
                mv = float(bat_st["voltages"][0])
                if 1000.0 < mv < 65000.0:
                    voltage_v = mv / 1000.0

        if voltage_v is None or not (6.0 <= voltage_v <= 26.0):
            return self.BASE_HOVER_THRUST

        ratio = self.NOMINAL_VOLTAGE_V / voltage_v
        compensated = self.BASE_HOVER_THRUST * (ratio ** 1.5)
        self._current_hover_ff = max(self.MIN_HOVER_FF, min(self.MAX_HOVER_FF, compensated))
        return self._current_hover_ff

    def _get_current_altitude_m(self) -> Optional[float]:
        if not self.distance:
            return None

        if hasattr(self.distance, "is_healthy") and not self.distance.is_healthy():
            return None

        state = getattr(self.distance, "state", self.distance)
        if getattr(state, "sensor_status", "OK") != "OK":
            return None

        dist_m = getattr(state, "distance_m", None)
        min_m = float(getattr(state, "min_distance_m", 0.04))
        max_m = float(getattr(state, "max_distance_m", 4.0))

        if isinstance(dist_m, (int, float)) and min_m <= float(dist_m) <= max_m:
            return float(dist_m)
        return None

    def _read_mavlink_imu_kinematics(self) -> Tuple[Optional[float], Optional[float]]:
        """Reads fast 50Hz Pixhawk EKF vertical velocity (LOCAL_POSITION_NED) and IMU acceleration."""
        if not self.mavlink or not hasattr(self.mavlink, "state"):
            return None, None

        mstate = self.mavlink.state
        get_pkt = getattr(mstate, "get_packet", None)
        if not callable(get_pkt):
            return None, None

        # Prefer high-rate LOCAL_POSITION_NED.vz (negated because NED is positive down)
        mav_vz_up: Optional[float] = None
        local_pos = get_pkt("LOCAL_POSITION_NED")
        if isinstance(local_pos, dict) and isinstance(local_pos.get("vz"), (int, float)):
            mav_vz_up = -float(local_pos["vz"])
        else:
            vfr = get_pkt("VFR_HUD")
            if isinstance(vfr, dict) and isinstance(vfr.get("climb"), (int, float)):
                mav_vz_up = float(vfr["climb"])

        att = get_pkt("ATTITUDE") or {}
        roll = float(att.get("roll", 0.0)) if isinstance(att, dict) else 0.0
        pitch = float(att.get("pitch", 0.0)) if isinstance(att, dict) else 0.0

        highres = get_pkt("HIGHRES_IMU")
        raw_imu = get_pkt("RAW_IMU")

        fx = fy = fz = None
        imu_sig = None

        if isinstance(highres, dict) and all(k in highres for k in ("xacc", "yacc", "zacc")):
            fx = float(highres["xacc"])
            fy = float(highres["yacc"])
            fz = float(highres["zacc"])
            imu_sig = ("HIGHRES", highres.get("time_usec"), round(fx, 4), round(fy, 4), round(fz, 4))
        elif isinstance(raw_imu, dict) and all(k in raw_imu for k in ("xacc", "yacc", "zacc")):
            scale = self.GRAVITY_MSS / 1000.0
            fx = float(raw_imu["xacc"]) * scale
            fy = float(raw_imu["yacc"]) * scale
            fz = float(raw_imu["zacc"]) * scale
            imu_sig = ("RAW", raw_imu.get("time_usec"), raw_imu["xacc"], raw_imu["yacc"], raw_imu["zacc"])

        if fx is None or imu_sig == self._last_imu_sig:
            return None, mav_vz_up

        self._last_imu_sig = imu_sig

        f_earth_down = (
            -fx * math.sin(pitch)
            + fy * math.sin(roll) * math.cos(pitch)
            + fz * math.cos(roll) * math.cos(pitch)
        )
        az_up = -(f_earth_down + self.GRAVITY_MSS)
        az_up = max(-15.0, min(15.0, az_up))
        return az_up, mav_vz_up

    # =========================================================================
    # COMPLEMENTARY ESTIMATOR & HIGH-RESPONSE ANALOG THRUST CONTROLLER
    # =========================================================================

    def _reset_vertical_controller(self, initial_alt: Optional[float] = None):
        now = time.monotonic()
        self._est_z = float(initial_alt) if initial_alt is not None else 0.0
        self._est_vz = 0.0
        self._prev_est_vz = 0.0
        self._az_bias = 0.0
        self._last_az_corr = 0.0
        self._estimator_initialized = initial_alt is not None
        self._last_est_time = now
        self._last_imu_sig = None
        self._last_lidar_val = initial_alt
        self._last_valid_lidar_time = now if initial_alt is not None else 0.0
        self._lidar_fault_active = False

        self._vel_integral = 0.0
        self._stable_since = None
        self._current_hover_ff = self._get_dynamic_hover_thrust()
        self.last_thrust = self._current_hover_ff

    def _clamp_thrust(self, thrust: float) -> float:
        return max(self.MIN_THRUST, min(self.MAX_THRUST, thrust))

    def _publish_telemetry_state(
        self,
        vz_sp: float,
        p_term: float,
        d_term: float,
        cur_lidar: Optional[float],
    ):
        target_z = self.setpoint[2] if self.setpoint[2] > 0.05 else self.target_altitude_m
        if cur_lidar is not None and self._last_imu_sig is not None:
            src = "IMU+LIDAR"
        elif self._last_imu_sig is not None:
            src = "IMU_ONLY"
        elif cur_lidar is not None:
            src = "LIDAR_ONLY"
        else:
            src = "OPEN_LOOP"

        self.state.update_control_telemetry(
            phase=self.phase.value,
            active_guidance=self.active_guidance,
            mission_mode=self.mission_mode,
            sensor_source=src,
            thrust=self.last_thrust,
            hover_trim=self._current_hover_ff + self._vel_integral,
            p_term=p_term,
            i_term=self._vel_integral,
            d_term=d_term,
            z_est_m=self._est_z,
            z_ref_m=target_z,
            vz_est_mps=self._est_vz,
            vz_ref_mps=vz_sp,
            lidar_m=cur_lidar,
            imu_az_mps2=self._last_az_corr,
            x_est_m=self._x_est_m,
            y_est_m=self._y_est_m,
            x_ref_m=self.setpoint[0],
            y_ref_m=self.setpoint[1],
            roll_deg=self._cmd_roll_deg,
            pitch_deg=self._cmd_pitch_deg,
            yaw_deg=self._cmd_yaw_deg,
        )

    def _compute_cascaded_thrust(self, now: float) -> Tuple[float, Optional[float], bool]:
        if self._last_est_time is None:
            dt = 1.0 / max(1.0, self.target_hz)
        else:
            dt = max(1e-3, min(0.10, now - self._last_est_time))
        self._last_est_time = now

        hover_ff = self._get_dynamic_hover_thrust()

        # 1. Fast 50Hz Pixhawk IMU + EKF3 vertical velocity fusion
        fresh_az_up, mav_vz_up = self._read_mavlink_imu_kinematics()
        if fresh_az_up is not None:
            a_corr = fresh_az_up - self._az_bias
            self._last_az_corr = a_corr
            self._est_vz += a_corr * dt
        else:
            a_corr = self._last_az_corr * 0.80
            self._last_az_corr = a_corr

        # Trust Pixhawk's onboard 400Hz EKF3 vertical velocity strongly (zero-latency climb/descent)
        if mav_vz_up is not None:
            self._est_vz = 0.40 * self._est_vz + 0.60 * mav_vz_up
        elif fresh_az_up is None:
            self._est_vz *= 0.96

        self._est_z += self._est_vz * dt

        # 2. VL53L1X external reference correction (30Hz)
        cur_lidar = self._get_current_altitude_m()
        if cur_lidar is not None:
            self._last_valid_lidar_time = now
            if self._lidar_fault_active:
                self._lidar_fault_active = False
                print("[Control] VL53L1X reference RESTORED.")

            if not self._estimator_initialized:
                self._est_z = cur_lidar
                self._est_vz = mav_vz_up if mav_vz_up is not None else 0.0
                self._estimator_initialized = True
            else:
                is_new_sample = (self._last_lidar_val is None) or (abs(cur_lidar - self._last_lidar_val) > 1e-4)
                kz = self.OBS_KZ if is_new_sample else (self.OBS_KZ * 0.30)
                kv = self.OBS_KV if is_new_sample else (self.OBS_KV * 0.30)

                err_z = cur_lidar - self._est_z
                self._est_z += kz * err_z
                self._est_vz += kv * err_z
                self._az_bias = max(-1.5, min(1.5, self._az_bias - self.OBS_KB * err_z * dt))

            self._last_lidar_val = cur_lidar
        else:
            lost_sec = (now - self._last_valid_lidar_time) if self._last_valid_lidar_time > 0.0 else 999.0

            if not self._estimator_initialized:
                self.last_thrust = max(self.FALLBACK_CLIMB_THRUST, hover_ff + 0.06)
                self._publish_telemetry_state(vz_sp=0.30, p_term=0.0, d_term=0.0, cur_lidar=None)
                return self.last_thrust, None, False

            if lost_sec >= self.LIDAR_LAND_SEC:
                print(f"[Control] FAILSAFE: VL53L1X lost for {lost_sec:.1f}s -> Commanding safe LAND.")
                return hover_ff, self._est_z, True

            if lost_sec >= self.LIDAR_WARN_SEC and not self._lidar_fault_active:
                self._lidar_fault_active = True
                print(f"[Control] WARN: VL53L1X lost for {lost_sec:.1f}s -> IMU-damped controlled descent.")

        self._est_z = max(0.0, self._est_z)

        # 3. Outer Position Loop -> Desired vertical velocity (vz_sp)
        target_z = self.setpoint[2] if self.setpoint[2] > 0.05 else self.target_altitude_m
        if self._lidar_fault_active:
            vz_sp = -0.25
        else:
            z_err = target_z - self._est_z
            if abs(z_err) <= self.POS_DEADBAND_M:
                vz_sp = 0.0
            else:
                vz_sp = max(-self.MAX_DESCENT_VZ, min(self.MAX_CLIMB_VZ, self.POS_KP * z_err))

        # 4. Fast 50Hz Inner Loop -> High-authority Analog Thrust
        vz_err = vz_sp - self._est_vz
        p_vel = self.VEL_KP * vz_err

        dvz_dt = (self._est_vz - self._prev_est_vz) / dt
        self._prev_est_vz = self._est_vz
        d_term = -(self.ACC_KD * a_corr + self.VEL_KD * dvz_dt)

        # Fast integral bleed when error flips sign
        if (vz_err > 0.0 and self._vel_integral < 0.0) or (vz_err < 0.0 and self._vel_integral > 0.0):
            self._vel_integral *= 0.80

        candidate_i = max(-self.VEL_I_MAX, min(self.VEL_I_MAX, self._vel_integral + self.VEL_KI * vz_err * dt))
        raw_thrust = hover_ff + p_vel + candidate_i + d_term
        target_thrust = self._clamp_thrust(raw_thrust)

        if raw_thrust == target_thrust or (raw_thrust > target_thrust and vz_err < 0.0) or (raw_thrust < target_thrust and vz_err > 0.0):
            self._vel_integral = candidate_i

        self.last_thrust = (
            self.THRUST_SMOOTH_ALPHA * target_thrust
            + (1.0 - self.THRUST_SMOOTH_ALPHA) * self.last_thrust
        )
        self.last_thrust = self._clamp_thrust(self.last_thrust)

        self._publish_telemetry_state(
            vz_sp=vz_sp,
            p_term=p_vel,
            d_term=d_term,
            cur_lidar=cur_lidar,
        )
        return self.last_thrust, self._est_z, False

    # =========================================================================
    # 50 HZ CLOSED-LOOP EXECUTION ENGINE
    # =========================================================================

    def _control_loop(self):
        interval = 1.0 / max(1.0, self.target_hz)
        while self.running:
            if not self.active_guidance or not self.mavlink:
                cur_lidar = self._get_current_altitude_m()
                if cur_lidar is not None:
                    self._est_z = cur_lidar
                self._get_dynamic_hover_thrust()
                self._publish_telemetry_state(vz_sp=0.0, p_term=0.0, d_term=0.0, cur_lidar=cur_lidar)
                time.sleep(0.1)
                continue

            if not self.mavlink.is_healthy():
                self.mission_mode = False
                self.active_guidance = False
                self.phase = MissionPhase.IDLE
                self._reset_vertical_controller()
                time.sleep(interval)
                continue

            now = time.monotonic()

            if self.phase == MissionPhase.ARMING:
                self._execute_arming()

            elif self.phase == MissionPhase.CLIMBING:
                thrust, fused_alt, trigger_land = self._compute_cascaded_thrust(now)
                if trigger_land:
                    self.command_land()
                    time.sleep(interval)
                    continue

                self.mavlink.send_attitude_target(self._cmd_roll_deg, self._cmd_pitch_deg, self._cmd_yaw_deg, thrust)

                target_z = self.setpoint[2] if self.setpoint[2] > 0.05 else self.target_altitude_m
                if fused_alt is not None and not self._lidar_fault_active:
                    alt_err = abs(target_z - fused_alt)
                    if alt_err <= self.ALT_TOLERANCE_M and abs(self._est_vz) <= self.VZ_STABLE_THRESH:
                        if self._stable_since is None:
                            self._stable_since = now
                        elif (now - self._stable_since) >= self.STABLE_HOLD_SEC:
                            next_phase = MissionPhase.ALIGNING if self.mission_mode else MissionPhase.HOVER
                            print(
                                f"[Control] Reference stabilized -> {next_phase.value} "
                                f"(z_est={fused_alt:.2f}m, ref={target_z:.2f}m, thrust={thrust:.3f})"
                            )
                            self.phase = next_phase
                    else:
                        self._stable_since = None

            elif self.phase in {MissionPhase.HOVER, MissionPhase.ALIGNING}:
                thrust, _, trigger_land = self._compute_cascaded_thrust(now)
                if trigger_land:
                    self.command_land()
                    time.sleep(interval)
                    continue

                self.mavlink.send_attitude_target(self._cmd_roll_deg, self._cmd_pitch_deg, self._cmd_yaw_deg, thrust)

            elif self.phase == MissionPhase.LANDING:
                self.command_land()

            time.sleep(interval)

    def _execute_arming(self):
        self.mavlink.set_mode("GUIDED_NOGPS")
        time.sleep(0.35)

        if not self.mavlink.state.is_armed():
            self.mavlink.arm()

        deadline = time.time() + 2.5
        while time.time() < deadline and self.running and self.active_guidance:
            if self.mavlink.state.is_armed():
                break
            time.sleep(0.05)

        if not self.mavlink.state.is_armed():
            print("[Control] Arming confirmation timed out -> IDLE")
            self.mission_mode = False
            self.active_guidance = False
            self.phase = MissionPhase.IDLE
            return

        init_alt = self._get_current_altitude_m()
        self._reset_vertical_controller(initial_alt=init_alt)
        thrust, _, _ = self._compute_cascaded_thrust(time.monotonic())
        print(
            f"[Control] Armed confirmed -> CLIMBING with 50Hz EKF/IMU+LiDAR Z control "
            f"(ref={self.target_altitude_m:.2f}m, hover_ff={self._current_hover_ff:.3f}, thrust={thrust:.3f})"
        )
        self.phase = MissionPhase.CLIMBING


MasterControl = ControlManager