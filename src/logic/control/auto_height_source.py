# file: src/logic/control/auto_height_source.py
"""Automatic EKF height source by flight mode (the pilot keeps choosing the MODES on the RC).

Flight pattern: take off in ALT_HOLD with BARO FLIGHT (height source baro), go over the table,
switch to LOITER (or GUIDED) on the RC. From then on the height must come from the lidar
(more precise), protected by the lidar spike gate (camera / baro referees), so that a table
edge, slot or obstacle under the lidar is never followed. Back in ALT_HOLD (or any other
mode) the height goes back to the barometer.

    armed, LOITER/GUIDED, source != LIDAR, lidar valid (gate says LIDAR) -> source set 1 (lidar)
    armed, other mode, we switched it this flight                       -> back to the take-off source
    disarmed, we switched it this flight                                -> back to the take-off source

Never changes the flight mode or any parameter: only MAV_CMD_SET_EKF_SOURCE_SET (42007),
waiting for the ACK. If the flight started on the lidar, nothing is ever changed."""

import threading
import time

LIDAR_SET = 1
AUTO_MODES = {4: "GUIDED", 5: "LOITER"}
MAV_CMD_SET_EKF_SOURCE_SET = 42007
MAV_RESULT_ACCEPTED = 0


class AutoHeightSource(threading.Thread):
    PERIOD_S = 0.2
    ACK_TIMEOUT_S = 1.5
    RETRY_S = 2.0
    MAX_TRIES = 3

    def __init__(self, mavlink_node, distance_state, log=print):
        super().__init__(name="AutoHeightSource", daemon=True)
        self.mav = mavlink_node
        self.dist = distance_state
        self.log = log
        self.enabled = True
        self.running = False
        self._was_armed = False
        self.base = None            # height source set at arming
        self.switched = False       # we put the lidar this flight
        self._want = None           # (set, reason) being requested
        self._tries = 0
        self._next_try = 0.0
        self._waiting_logged = False

    # ------------------------------------------------------------------ inputs
    def _state(self):
        vs = self.mav.state
        hb = vs.get_packet("HEARTBEAT") or {}
        return bool(vs.is_armed()), int(hb.get("custom_mode", -1)), int(vs.get_height_source_set() or 0)

    def lidar_ok(self):
        st = self.dist
        if st is None or getattr(st, "sensor_status", "") != "OK" or not getattr(st, "connected", True):
            return False
        gate = getattr(st, "gate", None)
        if gate is not None and getattr(gate, "source", "LIDAR") != "LIDAR":
            return False               # gate is correcting a slot/edge right now: wait
        h = float(getattr(st, "distance_m", 0.0) or 0.0)
        return float(getattr(st, "min_distance_m", 0.04)) <= h <= float(getattr(st, "max_distance_m", 4.0))

    # ------------------------------------------------------------------ decision
    def target(self, armed, mode, cur):
        """(wanted source set, reason) or None. Updates the per-flight memory."""
        if armed and not self._was_armed:
            self.base, self.switched = cur, False
        if not armed:
            if self.switched and self.base and cur != self.base:
                return self.base, "disarmed"
            return None
        if mode in AUTO_MODES:
            if cur != LIDAR_SET:
                return LIDAR_SET, f"auto, {AUTO_MODES[mode]}"
            return None
        if self.switched and self.base and cur != self.base:
            return self.base, f"auto, mode {mode}"
        return None

    # ------------------------------------------------------------------ loop
    def run(self):
        self.running = True
        while self.running:
            try:
                self.tick()
            except Exception as e:  # noqa: BLE001
                self.log(f"autoheight error: {e}", "ERROR")
            time.sleep(self.PERIOD_S)

    def tick(self):
        if not self.enabled or not getattr(self.mav, "connected", True):
            return
        armed, mode, cur = self._state()
        want = self.target(armed, mode, cur)
        self._was_armed = armed
        if want is None:
            self._want, self._tries, self._waiting_logged = None, 0, False
            return
        if want != self._want:
            self._want, self._tries, self._next_try = want, 0, 0.0
        if self._tries >= self.MAX_TRIES or time.time() < self._next_try:
            return
        source_set, reason = want
        if source_set == LIDAR_SET and not self.lidar_ok():
            if not self._waiting_logged:
                self.log(f"autoheight: {reason}: waiting for a valid lidar reading before switching "
                         "the height source to LIDAR", "WARN")
                self._waiting_logged = True
            return
        self._request(source_set, reason)

    def _request(self, source_set, reason):
        vs = self.mav.state
        name = vs.HEIGHT_SOURCE_NAMES.get(source_set, source_set)
        self._tries += 1
        seq = vs.last_event_seq() if hasattr(vs, "last_event_seq") else 0
        if not self.mav.set_ekf_source_set(source_set):
            self._fail(name, reason, "could not send")
            return
        deadline = time.time() + self.ACK_TIMEOUT_S
        result = None
        while time.time() < deadline and result is None:
            for _s, _t, kind, data in vs.get_events_since(seq):
                if kind == "COMMAND_ACK" and int(data.get("command", -1)) == MAV_CMD_SET_EKF_SOURCE_SET:
                    result = int(data.get("result", -1))
                    break
            time.sleep(0.05)
        if result == MAV_RESULT_ACCEPTED:
            vs.set_height_source_set(source_set)
            self.switched = source_set == LIDAR_SET and self.base != LIDAR_SET
            self._want, self._tries, self._waiting_logged = None, 0, False
            self.log(f"Height source -> {name} ({reason})", "OK")
        else:
            self._fail(name, reason, "no ACK" if result is None else f"result {result}")

    def _fail(self, name, reason, why):
        self._next_try = time.time() + self.RETRY_S
        left = self.MAX_TRIES - self._tries
        self.log(f"autoheight: height source -> {name} ({reason}) FAILED: {why}"
                 + (f", retrying ({left} left)" if left > 0 else ", giving up until the mode changes"), "ERROR")

    def status_text(self):
        vs = self.mav.state
        cur = vs.HEIGHT_SOURCE_NAMES.get(vs.get_height_source_set(), "?")
        base = vs.HEIGHT_SOURCE_NAMES.get(self.base, "-") if self.base else "-"
        return (f"autoheight {'ON' if self.enabled else 'OFF'} | height source now {cur} | "
                f"take-off source {base} | switched to lidar this flight: {self.switched}")
