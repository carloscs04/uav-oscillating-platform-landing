# file: src/interfaces/MAVLink/flow_check.py
"""No-flight checks for optical-flow Loiter (props OFF, vehicle DISARMED).

    flowcheck        snapshot: Pixhawk parameters vs ardupilot_flow_loiter.param,
                     Pi flow + rangefinder, what the Pixhawk reports back, EKF flags
    flowtest [s]     rotation test (default 15 s): rock the drone in roll and pitch
                     over a textured floor; compares the camera flow with the
                     Pixhawk gyro -> axis mapping, sign, scale (FLOW_F?SCALER) and
                     delay (EK3_FLOW_DELAY)
    flowwatch [s]    live line every second (for the in-hand walk test)

Everything here only READS state; nothing is sent to the Pixhawk except
parameter read requests.
"""

import os
import threading
import time
from typing import Any, Callable, List, Optional, Tuple

import numpy as np

PARAM_FILE = os.path.join(os.path.dirname(__file__), "ardupilot_flow_loiter.param")

# MAVLink parameter types (MAV_PARAM_TYPE) as ArduPilot declares them, checked in
# the ArduPilot source (Copter-4.5 and master): AP_Int8 / AP_Enum -> 2 (INT8),
# AP_Int16 -> 4 (INT16), AP_Int32 -> 6 (INT32), AP_UInt32 -> 5. QGroundControl
# skips any parameter whose type in the file differs from the vehicle's.
MAV_PARAM_TYPES = {
    "FLOW_FXSCALER": 4, "FLOW_FYSCALER": 4, "FLOW_ORIENT_YAW": 4,
    "ARMING_CHECK": 6, "ARMING_SKIPCHK": 5,
    # AP_Float (metres) in ArduPilot 4.6+
    "RNGFND1_MIN": 9, "RNGFND1_MAX": 9, "RNGFND1_GNDCLR": 9,
}
DEFAULT_MAV_PARAM_TYPE = 2   # every other parameter in the file is an 8-bit integer
QGC_FILE = os.path.join(os.path.dirname(__file__), "ardupilot_flow_loiter_qgc.params")


# Values that are tuned per vehicle: a difference from the file is not an error.
TUNED_PARAMS = {"EK3_FLOW_DELAY", "FLOW_FXSCALER", "FLOW_FYSCALER", "DISARM_DELAY"}

EKF_FLAGS = [
    (1, "attitude"), (2, "vel_horiz"), (4, "vel_vert"), (8, "pos_horiz_rel"),
    (16, "pos_horiz_abs"), (32, "pos_vert_abs"), (64, "pos_vert_agl"),
    (128, "const_pos_mode"), (256, "pred_pos_horiz_rel"), (512, "pred_pos_horiz_abs"),
    (1024, "uninitialized"),
]

Line = Tuple[str, str]   # (tag, text): PASS / WARN / FAIL / INFO


def load_expected(path: str = PARAM_FILE) -> List[Tuple[str, float]]:
    out = []
    with open(path) as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.replace(",", " ").split()
            if len(parts) >= 2:
                out.append((parts[0].upper(), float(parts[1])))
    return out


def export_qgc(src: str = PARAM_FILE, dst: str = QGC_FILE) -> str:
    """Write the .param file in QGroundControl's format (tab separated, with types)."""
    rows = [
        "# Onboard parameters for Vehicle 1",
        "#",
        f"# Generated from {os.path.basename(src)} by flow_check.export_qgc(). Edit that file, not this one.",
        "# Load in QGroundControl: Vehicle Setup > Parameters > Tools > Load from file, then reboot.",
        "#",
        "# Vehicle-Id Component-Id Name Value Type",
    ]
    for name, value in load_expected(src):
        ptype = MAV_PARAM_TYPES.get(name, DEFAULT_MAV_PARAM_TYPE)
        text = f"{value:.6f}" if ptype == 9 else str(int(round(value)))
        rows.append(f"1\t1\t{name}\t{text}\t{ptype}")
    with open(dst, "w") as f:
        f.write("\n".join(rows) + "\n")
    return dst


def decode_ekf_flags(flags: int) -> List[str]:
    return [name for bit, name in EKF_FLAGS if flags & bit]


def _fmt_age(age: Optional[float]) -> str:
    return "never" if age is None else f"{age:.1f}s ago"


# ---------------------------------------------------------------- flowcheck
def check_params(params: dict) -> List[Line]:
    lines: List[Line] = []
    for name, want in load_expected():
        have = params.get(name)
        if name == "ARMING_CHECK" and have is None and "ARMING_SKIPCHK" in params:
            skip = params["ARMING_SKIPCHK"]   # ArduPilot 4.7+: set bits = checks SKIPPED
            lines.append(("PASS" if skip == 0 else "FAIL",
                          f"ARMING_SKIPCHK = {skip:g} (ArduPilot 4.7+ replaces ARMING_CHECK; want 0 = run all checks)"))
            continue
        if have is None:
            lines.append(("WARN", f"{name}: not read (wait a few s and rerun, or not on this firmware); want {want:g}"))
        elif abs(float(have) - want) < 1e-3:
            lines.append(("PASS", f"{name} = {have:g}"))
        elif name in TUNED_PARAMS:
            lines.append(("INFO", f"{name} = {have:g} (file starting value {want:g}; tuned values are fine)"))
        else:
            lines.append(("FAIL", f"{name} = {have:g}, want {want:g}"))
    return lines


def check_live(vehicle_state: Any, feed: Any) -> List[Line]:
    lines: List[Line] = []
    vs = getattr(feed, "vision_state", None)

    # --- Pi: camera flow
    flow_ts = float(getattr(vs, "last_flow_ts", 0.0)) if vs is not None else 0.0
    if vs is None:
        lines.append(("FAIL", "Pi: vision_state not bound to the MAVLink feed"))
    elif flow_ts == 0.0 or time.time() - flow_ts > 0.5:
        lines.append(("FAIL", f"Pi: no camera flow for {time.time() - flow_ts:.0f}s (start the camera node)"
                              if flow_ts else "Pi: no camera flow yet (start the camera node)"))
    else:
        lines.append(("PASS", f"Pi: camera flow live, quality {getattr(vs, 'flow_quality', 0)}"))

    # --- Pi: rangefinder
    lidar = feed._read_valid_lidar_m()
    if lidar is None:
        lines.append(("FAIL", "Pi: no valid rangefinder reading (flow is sent with quality 0 without it)"))
    else:
        tag = "PASS" if lidar >= feed.MIN_FLOW_HEIGHT_M else "INFO"
        lines.append((tag, f"Pi: rangefinder {lidar:.2f} m" +
                      ("" if lidar >= feed.MIN_FLOW_HEIGHT_M else
                       f" (< {feed.MIN_FLOW_HEIGHT_M} m: flow sent as quality 0, lift it to test)")))

    # --- Pi: what was sent
    lf = getattr(feed, "last_flow", {}) or {}
    if not lf or time.time() - lf.get("t", 0) > 0.5:
        lines.append(("FAIL", f"Pi: no OPTICAL_FLOW sent recently (total sent {feed.flow_packets_sent})"))
    else:
        lines.append(("PASS", f"Pi: sending OPTICAL_FLOW (msg 100) ~{feed.actual_rate_hz:.0f} Hz, "
                              f"rate=({lf['rate_x']:+.3f}, {lf['rate_y']:+.3f}) rad/s q={lf['quality']}"
                              f"{' [gated: too low]' if lf.get('gated') else ''}"))
    lines.append(("PASS" if not feed.SEND_EXTERNAL_NAV else "WARN",
                  f"Pi: ExternalNav (VISION_POSITION_ESTIMATE) {'OFF' if not feed.SEND_EXTERNAL_NAV else 'ON - not recommended'}"))

    # --- Pixhawk: does it receive and use the flow?
    of, of_t = vehicle_state.get_packet_with_time("OPTICAL_FLOW")
    if of is None or time.time() - of_t > 2.0:
        lines.append(("FAIL", f"Pixhawk: not reporting OPTICAL_FLOW ({_fmt_age(None if of_t is None else time.time() - of_t)}). "
                              "Check FLOW_TYPE=5 and reboot the Pixhawk after setting it."))
    else:
        q = of.get("quality", 0)
        rx, ry = of.get("flow_rate_x", 0.0), of.get("flow_rate_y", 0.0)
        lines.append(("PASS" if q > 0 else "WARN",
                      f"Pixhawk: flow received, quality {q}, rate=({rx:+.3f}, {ry:+.3f}) rad/s, "
                      f"hagl {of.get('ground_distance', 0.0):.2f} m" +
                      ("" if q > 0 else f" (quality 0: check the rangefinder (>= {feed.MIN_FLOW_HEIGHT_M} m) and floor texture/light)")))

    rng = vehicle_state.get_packet("RANGEFINDER")
    ds, ds_t = vehicle_state.get_packet_with_time("DISTANCE_SENSOR")
    if rng is not None:
        lines.append(("PASS", f"Pixhawk: rangefinder {rng.get('distance', 0.0):.2f} m"))
    elif ds is not None and time.time() - ds_t < 2.0:
        lines.append(("PASS", f"Pixhawk: rangefinder {ds.get('current_distance', 0) / 100:.2f} m"))
    else:
        lines.append(("WARN", "Pixhawk: no RANGEFINDER/DISTANCE_SENSOR report (check RNGFND1_TYPE=10, reboot)"))

    # --- EKF
    ekf, ekf_t = vehicle_state.get_packet_with_time("EKF_STATUS_REPORT")
    if ekf is None:
        lines.append(("WARN", "EKF: no EKF_STATUS_REPORT received"))
    else:
        flags = int(ekf.get("flags", 0))
        names = decode_ekf_flags(flags)
        rel = ("pos_horiz_rel" in names or "pred_pos_horiz_rel" in names) and "const_pos_mode" not in names
        lines.append(("PASS" if rel else "WARN",
                      f"EKF: flags {flags} [{', '.join(names) or 'none'}] -> "
                      f"{'relative position OK (Loiter possible)' if rel else 'no relative position (Loiter would be refused)'}"))
        lines.append(("INFO", f"EKF: variances vel {ekf.get('velocity_variance', 0):.2f} "
                              f"pos_h {ekf.get('pos_horiz_variance', 0):.2f} pos_v {ekf.get('pos_vert_variance', 0):.2f} "
                              f"(below 0.5 is good, above 1.0 fails checks)"))

    lp = vehicle_state.get_packet("LOCAL_POSITION_NED")
    if lp:
        lines.append(("INFO", f"EKF local position x={lp.get('x', 0):+.2f} y={lp.get('y', 0):+.2f} "
                              f"z={lp.get('z', 0):+.2f} m  v=({lp.get('vx', 0):+.2f}, {lp.get('vy', 0):+.2f}) m/s"))
    st = vehicle_state.get_packet("STATUSTEXT")
    if st and st.get("text"):
        lines.append(("INFO", f"Last Pixhawk message: {st['text']}"))
    return lines


def verdict(lines: List[Line]) -> Line:
    fails = [t for tag, t in lines if tag == "FAIL"]
    if fails:
        return ("FAIL", f"NOT READY: {len(fails)} problem(s) above")
    warns = [t for tag, t in lines if tag == "WARN"]
    if warns:
        return ("WARN", f"Check the {len(warns)} warning(s) above, then run 'flowtest'")
    return ("PASS", "All checks pass. Next: 'flowtest' (rotation test), then the walk test")


# ---------------------------------------------------------------- flowtest
def _best_lag_corr(t, f, g, max_lag_s=0.25, step_s=0.01):
    """Correlation of flow f(t) with gyro g(t - lag) for lag in [0, max_lag]."""
    best = (0.0, 0.0, 0.0)  # corr, lag, gain
    for lag in np.arange(0.0, max_lag_s + 1e-9, step_s):
        n = int(round(lag / step_s))
        ff, gg = (f[n:], g[:len(g) - n]) if n else (f, g)
        if len(ff) < 50 or np.std(ff) < 1e-6 or np.std(gg) < 1e-6:
            continue
        c = float(np.corrcoef(ff, gg)[0, 1])
        if abs(c) > abs(best[0]):
            gain = float(np.dot(ff - ff.mean(), gg - gg.mean()) / np.dot(gg - gg.mean(), gg - gg.mean()))
            best = (c, float(lag), gain)
    return best


def analyse_rotation(gyro: np.ndarray, flow: np.ndarray, step_s=0.01) -> dict:
    """gyro: rows (t, roll_rate, pitch_rate); flow: rows (t, rate_x, rate_y)."""
    t0, t1 = max(gyro[0, 0], flow[0, 0]), min(gyro[-1, 0], flow[-1, 0])
    t = np.arange(t0, t1, step_s)
    gx, gy = np.interp(t, gyro[:, 0], gyro[:, 1]), np.interp(t, gyro[:, 0], gyro[:, 2])
    fx, fy = np.interp(t, flow[:, 0], flow[:, 1]), np.interp(t, flow[:, 0], flow[:, 2])
    return {
        "duration": float(t1 - t0), "gyro_std": (float(np.std(gx)), float(np.std(gy))),
        "xx": _best_lag_corr(t, fx, gx), "yy": _best_lag_corr(t, fy, gy),
        "xy": _best_lag_corr(t, fx, gy), "yx": _best_lag_corr(t, fy, gx),
    }


def report_rotation(res: dict, title: str, raw: bool) -> List[Line]:
    lines: List[Line] = [("INFO", f"--- {title} ({res['duration']:.0f} s of data)")]
    sx, sy = res["gyro_std"]
    if sx < 0.3 or sy < 0.3:
        lines.append(("WARN", f"Not enough rotation (roll {sx:.2f}, pitch {sy:.2f} rad/s RMS; want > 0.3): "
                              "rock it more, about +-15 deg, 1-2 times per second, on BOTH axes"))
    cxx, lxx, gxx = res["xx"]
    cyy, lyy, gyy = res["yy"]
    cxy, cyx = res["xy"][0], res["yx"][0]
    lines.append(("INFO", f"corr flowX~rollrate {cxx:+.2f}  flowY~pitchrate {cyy:+.2f}  "
                          f"(cross: flowX~pitch {cxy:+.2f}, flowY~roll {cyx:+.2f})"))
    if abs(cxy) + abs(cyx) > abs(cxx) + abs(cyy):
        lines.append(("FAIL", "Axes are SWAPPED: toggle optical_flow.swap_xy in camera_driver/config.yaml, "
                              "restart, and run flowtest again"))
        return lines
    for axis, c, inv in (("X", cxx, "invert_x"), ("Y", cyy, "invert_y")):
        if abs(c) < 0.6:
            lines.append(("FAIL", f"flow{axis}: weak correlation {c:+.2f} (want > 0.8). Check texture/light, "
                                  "height 0.5-1 m, camera focus and mounting"))
        elif c < 0:
            lines.append(("FAIL", f"flow{axis}: sign is INVERTED: toggle optical_flow.{inv} in camera_driver/config.yaml"))
        else:
            lines.append(("PASS", f"flow{axis}: sign correct (corr {c:+.2f})"))
    for axis, c, gain, param in (("X", cxx, gxx, "FLOW_FXSCALER"), ("Y", cyy, gyy, "FLOW_FYSCALER")):
        if c > 0.6 and gain > 0:
            if raw:
                sc = int(round((1.0 / gain - 1.0) * 1000.0))
                tag = "PASS" if abs(sc) <= 100 else ("WARN" if abs(sc) <= 800 else "FAIL")
                lines.append((tag, f"flow{axis}: scale {gain:.2f}x gyro -> suggested {param} = {sc}"
                                   + ("" if abs(sc) <= 800 else " (out of range: check focal length / calibration)")
                                   + " (valid only if rotated about the camera; hand-held rocking that also "
                                     "slides the camera reads high: confirm with a flight log before applying)"))
            else:
                lines.append(("PASS" if 0.85 <= gain <= 1.15 else "WARN",
                              f"flow{axis} as used by the Pixhawk: {gain:.2f}x gyro (want 0.85-1.15)"))
    if cxx > 0.6 and cyy > 0.6:
        delay_ms = int(round(1000.0 * (lxx + lyy) / 2.0))
        if raw:
            lines.append(("INFO", f"Delay camera->Pixhawk ~{delay_ms} ms -> suggested EK3_FLOW_DELAY = {min(delay_ms, 127)}"
                                  + (" (above the 127 ms limit: reduce Pi latency)" if delay_ms > 127 else "")))
    return lines


class FlowTester:
    """Collects gyro + flow for a rotation test in a background thread."""

    def __init__(self, vehicle_state: Any, feed: Any):
        self.vehicle_state = vehicle_state
        self.feed = feed
        self.running = False

    def _collect(self, seconds: float):
        gyro, fc_flow = [], []
        last_att_ms, last_of_us = None, None
        t_start = time.time()
        while time.time() - t_start < seconds:
            att, att_t = self.vehicle_state.get_packet_with_time("ATTITUDE")
            if att is not None and att.get("time_boot_ms") != last_att_ms:
                last_att_ms = att.get("time_boot_ms")
                gyro.append((att_t, float(att.get("rollspeed", 0.0)), float(att.get("pitchspeed", 0.0))))
            of, of_t = self.vehicle_state.get_packet_with_time("OPTICAL_FLOW")
            if of is not None and of.get("time_usec") != last_of_us and of_t >= t_start:
                last_of_us = of.get("time_usec")
                fc_flow.append((of_t, float(of.get("flow_rate_x", 0.0)), float(of.get("flow_rate_y", 0.0))))
            time.sleep(0.005)
        pi_flow = [(t, rx, ry) for (t, rx, ry, q, h, gated) in list(self.feed.flow_history)
                   if t >= t_start and q > 0]
        return np.array(gyro), np.array(pi_flow), np.array(fc_flow)

    def run(self, seconds: float, log: Callable[[str, str], None]):
        if self.running:
            log("flowtest already running", "WARN")
            return
        self.running = True
        try:
            log(f"flowtest: ROCK the drone now for {seconds:.0f} s (roll AND pitch, +-15 deg, "
                "about 0.5-1 m above a textured floor, props OFF). Pivot around the CAMERA "
                "lens (keep the lens in place): sliding the camera adds flow and inflates the scale", "WARN")
            gyro, pi_flow, fc_flow = self._collect(seconds)
            if len(gyro) < 50:
                log(f"flowtest: only {len(gyro)} ATTITUDE samples: is the Pixhawk streaming?", "FAIL")
                return
            if len(pi_flow) < 50:
                log(f"flowtest: only {len(pi_flow)} flow samples with quality > 0: camera running? "
                    "floor textured? lit?", "FAIL")
                return
            lines = report_rotation(analyse_rotation(gyro, pi_flow), "Pi camera flow vs Pixhawk gyro", raw=True)
            if len(fc_flow) >= 50:
                lines += report_rotation(analyse_rotation(gyro, fc_flow),
                                         "Flow as the Pixhawk uses it (after FLOW_* params)", raw=False)
            else:
                lines.append(("WARN", f"Pixhawk reported only {len(fc_flow)} OPTICAL_FLOW samples: "
                                      "cannot check the Pixhawk side (FLOW_TYPE=5? rebooted? rangefinder valid?)"))
            for tag, text in lines:
                log(text, tag)
        finally:
            self.running = False


def watch(vehicle_state: Any, feed: Any, seconds: float, log: Callable[[str, str], None],
          stop: Optional[threading.Event] = None):
    """One line per second for the walk test."""
    t_end = time.time() + seconds
    p0: Optional[float] = None   # baro pressure and lidar height at start
    h0 = 0.0
    while time.time() < t_end and not (stop and stop.is_set()):
        lf = getattr(feed, "last_flow", {}) or {}
        of = vehicle_state.get_packet("OPTICAL_FLOW") or {}
        lp = vehicle_state.get_packet("LOCAL_POSITION_NED") or {}
        ekf = vehicle_state.get_packet("EKF_STATUS_REPORT") or {}
        h = lf.get("height_m")
        rel = bool(int(ekf.get("flags", 0)) & (8 | 256))
        # Baro vs lidar, both as height gained since flowwatch started (baro zeroed on
        # the pressure at start, so it is valid in the in-hand lift test while disarmed;
        # start it with the drone still on the floor)
        sp = vehicle_state.get_packet("SCALED_PRESSURE") or {}
        p_now = float(sp.get("press_abs", 0.0) or 0.0)
        if p0 is None and p_now > 0 and h is not None:
            # Zero on the averaged reference (EMA over ~2 s while disarmed), not one raw
            # sample: a single reading is +/-0.2 m of noise and biases the whole run.
            ref = getattr(vehicle_state, "_baro_ref_hpa", None)
            p0, h0 = (ref if ref else p_now), h
        baro_txt = ""
        if p0 and p_now > 0:
            b = 44330.77 * (1.0 - (p_now / p0) ** 0.190263)
            baro_txt = f" baro={b:+.2f}m"
            if h is not None:
                baro_txt += f" lidar={h - h0:+.2f}m diff={b - (h - h0):+.2f}"
        log(f"h={h if h is not None else -1:.2f}m piQ={lf.get('raw_quality', 0):3d} "
            f"fcQ={of.get('quality', 0):3d} "
            f"EKF pos=({lp.get('x', 0):+.2f}, {lp.get('y', 0):+.2f}) v=({lp.get('vx', 0):+.2f}, {lp.get('vy', 0):+.2f}) "
            f"rel={'OK' if rel else 'NO'} | bench=({feed._pos_x_m:+.2f}, {feed._pos_y_m:+.2f}){baro_txt}", "STATUS")
        time.sleep(1.0)


if __name__ == "__main__":
    # Regenerate the QGroundControl file after editing ardupilot_flow_loiter.param:
    #   python -m src.interfaces.MAVLink.flow_check
    print(f"Wrote {export_qgc()}")

