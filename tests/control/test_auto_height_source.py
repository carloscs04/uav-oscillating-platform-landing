"""
test_auto_height_source.py - automatic EKF height source by flight mode (no hardware).

    python3 tests/control/test_auto_height_source.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.logic.control.auto_height_source import AutoHeightSource  # noqa: E402

ALT_HOLD, LOITER, GUIDED = 2, 5, 4


class FakeVS:
    HEIGHT_SOURCE_NAMES = {0: "UNKNOWN", 1: "LIDAR", 2: "BARO"}

    def __init__(self):
        self.armed, self.mode, self.src, self.events, self.sent = False, ALT_HOLD, 2, [], []
        self.accept = True

    def get_packet(self, _k):
        return {"custom_mode": self.mode}

    def is_armed(self):
        return self.armed

    def get_height_source_set(self):
        return self.src

    def set_height_source_set(self, v):
        self.src = v

    def last_event_seq(self):
        return len(self.events)

    def get_events_since(self, seq):
        return [e for e in self.events if e[0] > seq]


class FakeMav:
    connected = True

    def __init__(self):
        self.state = FakeVS()

    def set_ekf_source_set(self, v):
        vs = self.state
        vs.sent.append(v)
        vs.events.append((len(vs.events) + 1, 0, "COMMAND_ACK",
                          {"command": 42007, "result": 0 if vs.accept else 4}))
        return True


class FakeDist:
    sensor_status, connected, distance_m, min_distance_m, max_distance_m = "OK", True, 0.6, 0.04, 4.0

    class gate:
        source = "LIDAR"


def make():
    mav, logs = FakeMav(), []
    a = AutoHeightSource(mav, FakeDist(), log=lambda m, l="": logs.append(m))
    a.ACK_TIMEOUT_S = 0.2
    a.enabled = True
    return a, mav.state, logs


def test_pilot_pattern_alt_hold_loiter_alt_hold_land():
    a, vs, logs = make()
    vs.src = 2                       # BARO FLIGHT before take-off
    vs.armed, vs.mode = True, ALT_HOLD
    a.tick()
    assert vs.sent == []             # ALT_HOLD on baro: untouched
    vs.mode = LOITER
    a.tick()
    assert vs.src == 1 and vs.sent == [1] and "LOITER" in logs[-1]
    a.tick()
    assert vs.sent == [1]            # no repeats
    vs.mode = ALT_HOLD
    a.tick()
    assert vs.src == 2 and vs.sent == [1, 2]
    vs.mode = GUIDED
    a.tick()
    assert vs.src == 1
    vs.armed = False                 # landed from GUIDED
    a.tick()
    assert vs.src == 2 and vs.sent == [1, 2, 1, 2]


def test_waits_while_gate_corrects():
    a, vs, logs = make()
    vs.armed, vs.mode = True, LOITER
    a.dist.gate.source = "BAROMETER"
    a.tick()
    assert vs.sent == [] and "waiting" in logs[-1]
    a.dist.gate.source = "LIDAR"
    a.tick()
    assert vs.src == 1


def test_flight_started_on_lidar_never_changed():
    a, vs, _ = make()
    vs.src = 1
    for armed, mode in ((True, ALT_HOLD), (True, LOITER), (True, ALT_HOLD), (False, ALT_HOLD)):
        vs.armed, vs.mode = armed, mode
        a.tick()
    assert vs.sent == []


def test_disabled_does_nothing():
    a, vs, _ = make()
    a.enabled = False
    vs.armed, vs.mode = True, LOITER
    a.tick()
    assert vs.sent == []


def test_rejected_ack_retries_then_gives_up():
    a, vs, logs = make()
    a.RETRY_S = 0.0
    vs.accept = False
    vs.armed, vs.mode = True, LOITER
    for _ in range(6):
        a.tick()
    assert len(vs.sent) == 3 and vs.src == 2 and "giving up" in logs[-1]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("PASS", name)
