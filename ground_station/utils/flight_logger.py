# file: ground_station/utils/flight_logger.py
import csv
import os
import random
import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional, TextIO


class FlightExperimentLogger:
    """Automatic flight telemetry recorder triggered exclusively while the vehicle is armed."""

    FLIGHT_CODENAMES = [
        "falcon",
        "kestrel",
        "osprey",
        "harrier",
        "merlin",
        "peregrine",
        "albatross",
        "condor",
        "vortex",
        "zenith",
        "horizon",
        "altitude",
        "thermals",
        "vector",
        "beacon",
        "azimuth",
        "glide",
        "hover",
        "thrust",
        "aeris",
    ]

    ARDUPILOT_MODES = {
        0: "STABILIZE",
        1: "ACRO",
        2: "ALT_HOLD",
        3: "AUTO",
        4: "GUIDED",
        5: "LOITER",
        6: "RTL",
        7: "CIRCLE",
        9: "LAND",
        16: "POSHOLD",
        17: "BRAKE",
        20: "GUIDED_NOGPS",
    }

    CSV_HEADERS = [
        "codename",
        "timestamp_iso",
        "elapsed_s",
        "armed",
        "flight_mode",
        "custom_mode",
        # Local NED Position & Velocity (m, m/s)
        "pos_x",
        "pos_y",
        "pos_z",
        "vel_x",
        "vel_y",
        "vel_z",
        # Optical Flow metrics
        "flow_x",
        "flow_y",
        "flow_comp_mx",
        "flow_comp_my",
        "flow_quality",
        "flow_ground_dist",
        "flow_int_x",
        "flow_int_y",
        # RangeFinder / LiDAR
        "lidar_cm",
        # Attitude & Angular Rates (deg, deg/s)
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "rollspeed_dps",
        "pitchspeed_dps",
        "yawspeed_dps",
        # Navigation Controller Targets
        "nav_roll_deg",
        "nav_pitch_deg",
        "nav_bearing",
        "target_bearing",
        "wp_dist_m",
        "alt_error_m",
        # Motor PWM Outputs
        "motor_1",
        "motor_2",
        "motor_3",
        "motor_4",
        # EKF3 Variances & Flags
        "ekf_flags",
        "ekf_vel_var",
        "ekf_pos_h_var",
        "ekf_pos_v_var",
        "ekf_compass_var",
        "ekf_terrain_var",
        "ekf_age_ms",
        # RC Pilot Inputs (PWM)
        "rc1_roll",
        "rc2_pitch",
        "rc3_throttle",
        "rc4_yaw",
        "rc5_mode",
        # Vibration & Clipping
        "vib_x",
        "vib_y",
        "vib_z",
        "clip_0",
        "clip_1",
        "clip_2",
        "vib_age_ms",
        # Pi-side optical flow actually sent to the Pixhawk (raw camera, before gating)
        "pi_flow_rate_x",
        "pi_flow_rate_y",
        "pi_flow_quality",
        "pi_flow_sent_quality",
        "pi_flow_gated",
        "pi_lidar_m",
        "pi_lidar_raw_m",    # sensor before the spike gate (pi_lidar_m = gated, sent)
        "pi_lidar_gate",     # OK | REJECT_CAM | REJECT_EST | UNRELIABLE
        "pi_lidar_rejected", # samples rejected since the Pi started
        "pi_flow_samples",
        "pi_flow_age_ms",
        "pi_flow_tx_hz",
        # Barometer (Pixhawk SCALED_PRESSURE): height above the take-off point
        "height_src",
        "baro_rel_alt_m",
        "baro_press_hpa",
        "baro_temp_c",
        "baro_age_ms",
        # Power & System Health
        "voltage_v",
        "current_a",
        "battery_pct",
        "cpu_load_pct",
        "link_status",
        "link_rate_hz",
    ]

    def __init__(self, base_dir: Optional[str] = None):
        if base_dir is None:
            # <repo>/experiments/flights/<date>/ (next to experiments/vision/)
            repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
            base_dir = os.path.join(repo_root, "experiments", "flights")

        self.base_dir = base_dir
        self._lock = threading.Lock()
        self._is_recording = False
        self._arm_start_time: float = 0.0
        self._csv_file: Optional[TextIO] = None
        self._csv_writer: Optional[csv.writer] = None
        self._event_file: Optional[TextIO] = None
        self._current_csv_path: Optional[str] = None
        self._current_codename: str = ""
        self._last_statustext: str = ""
        self._last_statustext_seq: int = 0
        self._last_mode: int = -1
        self._row_count: int = 0
        # Pre-arm/arming texts are logged even while DISARMED (that is when the arm fails)
        self._pending_texts: list = []
        self._logged_params: Dict[str, Any] = {}
        self._logged_ctrl: Dict[str, Any] = {}

    def process_telemetry(self, packets: Dict[str, Any], health_info: Dict[str, Any]):
        """Evaluates arming state and records telemetry rows while armed."""
        hb = packets.get("heartbeat") or {}
        armed = bool(hb.get("armed", False))

        with self._lock:
            self._collect_statustexts(packets)
            if armed and not self._is_recording:
                self._start_session(hb, health_info)
            elif not armed and self._is_recording:
                self._stop_session()

            if self._is_recording:
                self._log_params(packets)
                self._log_control_params(packets)
                self._write_sample(packets, health_info)

    def _log_params(self, packets: Dict[str, Any]):
        """Pixhawk params in the events log: all at arm, then any late or changed value."""
        params = packets.get("pixhawk_params") or {}
        new = {k: v for k, v in params.items() if self._logged_params.get(k) != v}
        if not new:
            return
        label = "PARAMS_UPDATE" if self._logged_params else "PARAMS"
        self._log_event(f"{label}: " + " ".join(f"{k}={new[k]}" for k in sorted(new)))
        self._logged_params.update(new)

    def _log_control_params(self, packets: Dict[str, Any]):
        """Pi landing law / GUIDED sender params: all at arm, then any changed value."""
        params = packets.get("control_params") or {}
        new = {k: v for k, v in params.items() if self._logged_ctrl.get(k) != v}
        if not new:
            return
        label = "CONTROL_PARAMS_UPDATE" if self._logged_ctrl else "CONTROL_PARAMS"
        self._log_event(f"{label}: " + " ".join(f"{k}={new[k]}" for k in sorted(new)))
        self._logged_ctrl.update(new)

    def _start_session(self, hb: Dict[str, Any], health_info: Dict[str, Any]):
        now = datetime.now()
        date_folder = now.strftime("%Y-%m-%d")
        time_tag = now.strftime("%H-%M-%S")
        codename = random.choice(self.FLIGHT_CODENAMES)

        session_dir = os.path.join(self.base_dir, date_folder)
        os.makedirs(session_dir, exist_ok=True)

        csv_path = os.path.join(session_dir, f"flight_{time_tag}_{codename}.csv")
        event_path = os.path.join(session_dir, f"flight_{time_tag}_{codename}_events.log")

        self._csv_file = open(csv_path, mode="w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self.CSV_HEADERS)

        self._event_file = open(event_path, mode="w", encoding="utf-8")
        self._arm_start_time = time.time()
        self._is_recording = True
        self._current_csv_path = csv_path
        self._current_codename = codename
        self._last_statustext = ""
        self._last_mode = -1
        self._row_count = 0
        self._last_hsrc = None
        self._logged_params = {}
        self._logged_ctrl = {}

        # Texts from the last 30 s before arming (PreArm, EKF, ...) head the events log
        for ts, sev, text in self._pending_texts:
            if time.time() - ts < 30.0:
                self._log_event(f"(before arm) STATUSTEXT [sev={sev}]: {text}")
        self._pending_texts = []

        self._log_event(
            f"ARMED [{codename.upper()}] | Initial mode={self._mode_name(hb.get('custom_mode', -1))} | "
            f"Voltage={health_info.get('voltage_v', 0.0)}V | Link={health_info.get('status', 'UNKNOWN')}"
        )
        print(f"[FLIGHT_LOGGER] Recording started [{codename}] -> {csv_path}")

    def _stop_session(self):
        if not self._is_recording:
            return

        duration = round(time.time() - self._arm_start_time, 2)
        self._log_event(
            f"DISARMED [{self._current_codename.upper()}] | Duration={duration}s | "
            f"Total_samples={self._row_count}"
        )

        try:
            if self._csv_file:
                self._csv_file.flush()
                self._csv_file.close()
            if self._event_file:
                self._event_file.flush()
                self._event_file.close()
        except Exception:
            pass

        print(
            f"[FLIGHT_LOGGER] Recording saved [{self._current_codename}] "
            f"({self._row_count} samples, {duration}s) -> {self._current_csv_path}"
        )

        self._is_recording = False
        self._csv_file = None
        self._csv_writer = None
        self._event_file = None
        self._current_csv_path = None
        self._current_codename = ""

    def _write_sample(self, packets: Dict[str, Any], health_info: Dict[str, Any]):
        if not self._csv_writer:
            return

        now_ts = time.time()
        elapsed = round(now_ts - self._arm_start_time, 3)
        iso_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        hb = packets.get("heartbeat") or {}
        custom_mode = hb.get("custom_mode", -1)
        mode_str = self._mode_name(custom_mode)

        if custom_mode != self._last_mode:
            self._log_event(f"MODE_CHANGE -> {mode_str} ({custom_mode}) at t+{elapsed}s")
            self._last_mode = custom_mode

        if not packets.get("statustext_log"):
            # Older Pi software: only the latest STATUSTEXT is available
            st = packets.get("statustext") or {}
            st_text = str(st.get("text", "")).strip()
            if st_text and st_text != self._last_statustext:
                self._log_event(f"STATUSTEXT [sev={st.get('severity', -1)}]: {st_text}")
                self._last_statustext = st_text

        lpos = packets.get("local_position") or {}
        flow = packets.get("optical_flow") or {}
        dist = packets.get("distance_sensor") or {}
        att = packets.get("attitude") or {}
        nav = packets.get("nav_output") or {}
        servo = packets.get("servo_output") or {}
        ekf = packets.get("ekf_status") or {}
        rc = packets.get("rc_channels") or {}
        vib = packets.get("vibration") or {}
        sys_st = packets.get("sys_status") or {}
        pif = packets.get("pi_flow") or {}
        baro = packets.get("baro") or {}
        hsrc = packets.get("height_src") or {}
        if hsrc.get("name") and hsrc.get("name") != getattr(self, "_last_hsrc", None):
            self._log_event(f"HEIGHT_SOURCE -> {hsrc.get('name')} at t+{elapsed}s")
            self._last_hsrc = hsrc.get("name")

        row = [
            self._current_codename,
            iso_str,
            elapsed,
            int(bool(hb.get("armed", False))),
            mode_str,
            custom_mode,
            lpos.get("x", 0.0),
            lpos.get("y", 0.0),
            lpos.get("z", 0.0),
            lpos.get("vx", 0.0),
            lpos.get("vy", 0.0),
            lpos.get("vz", 0.0),
            flow.get("flow_x", 0),
            flow.get("flow_y", 0),
            flow.get("comp_m_x", 0.0),
            flow.get("comp_m_y", 0.0),
            flow.get("quality", 0),
            flow.get("ground_distance", 0.0),
            flow.get("int_x", 0.0),
            flow.get("int_y", 0.0),
            dist.get("current_cm", 0),
            att.get("roll", 0.0),
            att.get("pitch", 0.0),
            att.get("yaw", 0.0),
            att.get("rollspeed", 0.0),
            att.get("pitchspeed", 0.0),
            att.get("yawspeed", 0.0),
            nav.get("nav_roll", 0.0),
            nav.get("nav_pitch", 0.0),
            nav.get("nav_bearing", 0),
            nav.get("target_bearing", 0),
            nav.get("wp_dist", 0),
            nav.get("alt_error", 0.0),
            servo.get("m1", 0),
            servo.get("m2", 0),
            servo.get("m3", 0),
            servo.get("m4", 0),
            ekf.get("flags", 0),
            ekf.get("vel_var", 0.0),
            ekf.get("pos_h_var", 0.0),
            ekf.get("pos_v_var", 0.0),
            ekf.get("compass_var", 0.0),
            ekf.get("terrain_var", 0.0),
            ekf.get("age_ms", -1),
            rc.get("roll_ch1", 0),
            rc.get("pitch_ch2", 0),
            rc.get("throttle_ch3", 0),
            rc.get("yaw_ch4", 0),
            rc.get("mode_ch5", 0),
            vib.get("vib_x", 0.0),
            vib.get("vib_y", 0.0),
            vib.get("vib_z", 0.0),
            vib.get("clip_0", 0),
            vib.get("clip_1", 0),
            vib.get("clip_2", 0),
            vib.get("age_ms", -1),
            pif.get("rate_x", 0.0),
            pif.get("rate_y", 0.0),
            pif.get("quality", 0),
            pif.get("sent_quality", 0),
            pif.get("gated", 0),
            pif.get("lidar_m", -1.0),
            pif.get("lidar_raw_m", -1.0),
            pif.get("lidar_gate", ""),
            pif.get("lidar_rejected", 0),
            pif.get("samples", 0),
            pif.get("age_ms", -1),
            pif.get("tx_hz", 0.0),
            hsrc.get("name", "UNKNOWN"),
            baro.get("rel_alt_m", 0.0),
            baro.get("press_hpa", 0.0),
            baro.get("temp_c", 0.0),
            baro.get("age_ms", -1),
            sys_st.get("voltage_v", 0.0),
            sys_st.get("current_a", 0.0),
            sys_st.get("battery_pct", -1),
            sys_st.get("cpu_load_pct", 0.0),
            health_info.get("status", "UNKNOWN"),
            health_info.get("rate_hz", 0.0),
        ]

        self._csv_writer.writerow(row)
        self._row_count += 1

        if self._row_count % 20 == 0 and self._csv_file:
            self._csv_file.flush()

    def _collect_statustexts(self, packets: Dict[str, Any]):
        """Every new Pixhawk STATUSTEXT (the Pi sends the last 10 with a sequence number)."""
        log = packets.get("statustext_log")
        if not isinstance(log, list) or not log:
            return
        newest = max(int(e[0]) for e in log)
        if newest < self._last_statustext_seq:      # Pi restarted: sequence starts again
            self._last_statustext_seq = 0
        for entry in log:
            try:
                seq, sev, text = int(entry[0]), entry[1], str(entry[2]).strip()
            except (TypeError, ValueError, IndexError):
                continue
            if seq <= self._last_statustext_seq or not text:
                continue
            if self._is_recording:
                self._log_event(f"STATUSTEXT [sev={sev}]: {text}")
            else:
                self._pending_texts.append((time.time(), sev, text))
        self._last_statustext_seq = max(self._last_statustext_seq, newest)
        self._pending_texts = self._pending_texts[-30:]

    def _log_event(self, message: str):
        if self._event_file:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self._event_file.write(f"[{ts}] {message}\n")
            self._event_file.flush()

    def _mode_name(self, custom_mode: int) -> str:
        return self.ARDUPILOT_MODES.get(custom_mode, f"MODE_{custom_mode}")

    def close(self):
        """Ensures active log files are flushed and closed on shutdown."""
        with self._lock:
            if self._is_recording:
                self._stop_session()