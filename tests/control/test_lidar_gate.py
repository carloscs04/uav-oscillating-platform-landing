"""
test_lidar_gate.py - lidar spike gate on synthetic and recorded heights (no hardware).

    python3 tests/control/test_lidar_gate.py
"""

import csv
import glob
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.logic.control import lidar_gate as _lg  # noqa: E402
from src.logic.control.lidar_gate import LidarGateParams  # noqa: E402


def LidarGate(params=None):  # tests exercise the gate itself: enabled unless told otherwise
    return _lg.LidarGate(params or LidarGateParams(enabled=True))

DT = 0.033   # ~30 Hz


def run(seq, cam=None, vz=None):
    g, out = LidarGate(), []
    for i, h in enumerate(seq):
        c = cam[i] if cam else None
        out.append(g.step(h, i * DT, vz[i] if vz else 0.0, None if c is None else 1.077 * c - 0.025))
    return g, out


def test_short_spike_rejected():
    seq = [0.45] * 20 + [0.95] * 3 + [0.45] * 20
    g, out = run(seq)
    assert all(abs(o - 0.45) < 1e-6 for o in out), out
    assert g.status == "OK" and g.events == 1


def test_long_slot_with_camera_never_accepted():
    seq = [0.45] * 20 + [0.95] * 300 + [0.45] * 10            # 10 s hovering over the slot
    g, out = run(seq, cam=[0.45] * len(seq))
    assert all(o is not None and abs(o - 0.45) < 0.005 for o in out)


def test_real_step_confirmed_by_camera():
    seq = [1.05] * 20 + [0.50] * 20                            # flew over the table edge
    g, out = run(seq, cam=[1.05] * 20 + [0.50] * 20)
    assert abs(out[20] - 0.50) < 1e-6 and g.status == "OK"


def test_no_camera_estimate_then_unreliable_then_settle():
    p = LidarGateParams()
    n_est = int(p.est_max_s / DT)
    seq = [0.45] * 20 + [0.95] * (n_est + 100)
    g, out = run(seq)
    assert all(abs(o - 0.45) < 1e-6 for o in out[20:20 + n_est - 1])       # prediction
    unrel = [i for i, o in enumerate(out) if o is None]
    assert unrel and unrel[0] > 20 + n_est - 2                            # then no height
    assert out[-1] == 0.95                                                 # steady -> accepted


def test_prediction_follows_descent():
    vz = [-0.2] * 60
    seq = [0.60 - 0.2 * i * DT for i in range(60)]
    seq[30:33] = [1.2] * 3                                     # spike while descending
    _g, out = run(seq, vz=vz)
    for i in range(30, 33):
        assert abs(out[i] - (0.60 - 0.2 * i * DT)) < 0.01


def test_recorded_flights_no_false_rejects():
    """Every LOITER/GUIDED sample of the platform day (2026-11-23), with baro: nothing rejected."""
    root = Path(__file__).resolve().parents[2] / "experiments" / "flights" / "2026-11-23"
    files = sorted(f for f in glob.glob(str(root / "**" / "flight_*.csv"), recursive=True)
                   if not f.endswith("_plnd.csv"))
    if not files:
        print("  (no recorded flights here, skipped)")
        return
    total = rej = 0
    for f in files:
        g, prev_t = LidarGate(), None
        for r in csv.DictReader(open(f)):
            try:
                h, vz = float(r["pi_lidar_m"]), float(r["vel_z"])
                hh, mm, ss = r["timestamp_iso"].split(":")
                t = int(hh) * 3600 + int(mm) * 60 + float(ss)
            except (KeyError, ValueError):
                continue
            if r.get("armed") != "1" or h < 0:
                continue
            if r.get("custom_mode") not in ("4", "5"):     # the gate acts only in GUIDED/LOITER
                g.idle(h, t)
                continue
            try:
                pr = float(r["baro_press_hpa"])
                baro = 44330.0 * (1.0 - (pr / 1013.25) ** 0.1903) if pr > 0 else None
            except (KeyError, ValueError):
                baro = None
            g.step(h, t, -vz, None, baro)
            total += 1
        rej += g.rejected_total
    print(f"  recorded 2026-11-23: {total} samples, {rej} rejected")
    assert rej <= total * 0.001


def test_gradual_drift_corrected_by_camera():
    seq = [0.45] * 20 + [0.45 + 0.05 * i for i in range(1, 11)] + [0.95] * 30   # 5 cm/sample: no "jump"
    g, out = run(seq, cam=[0.45] * len(seq))
    assert all(o is not None and abs(o - 0.45) < 0.03 for o in out[24:]), out[20:30]
    assert g.source == "CAMERA"


def test_camera_noise_does_not_override_lidar():
    import random
    rnd = random.Random(2)
    seq = [0.50] * 200
    cam = [0.50 + rnd.gauss(0, 0.02) for _ in seq]
    g, out = run(seq, cam=cam)
    assert g.rejected_total == 0 and all(o == 0.50 for o in out)


def test_gradual_slot_without_camera_caught_by_baro():
    """Slot entering the lidar cone: +4 cm per sample (never a 10 cm 'jump'), drone still."""
    seq = [0.45] * 30 + [0.45 + 0.04 * i for i in range(1, 13)] + [0.93] * 30
    g = LidarGate()
    out = [g.step(h, i * DT, 0.0, None, 100.0) for i, h in enumerate(seq)]
    assert max(o for o in out if o is not None) < 0.62, max(out)
    assert all(o is not None and abs(o - 0.45) < 0.03 for o in out[-25:]), out[-25:]
    assert g.source == "BAROMETER" and g.pred_src == "baro"


def test_real_descent_followed_by_baro_not_rejected():
    """Drone descends 0.3 m/s from 1.0 m: lidar and baro move together -> all accepted."""
    seq = [1.0 - 0.3 * i * DT for i in range(80)]
    g = LidarGate()
    out = [g.step(h, i * DT, -0.3, None, 100.0 + (h - 1.0)) for i, h in enumerate(seq)]
    assert g.rejected_total == 0 and out == seq


def test_baro_holds_without_time_limit():
    """Over the table edge / an obstacle, no marker: the baro holds the height (never None),
    and the lidar takes over again as soon as it fits."""
    seq = [0.45] * 20 + [0.95] * int(20 / DT) + [0.46] * 10
    g = LidarGate()
    out = [g.step(h, i * DT, 0.0, None, 50.0) for i, h in enumerate(seq)]
    assert all(o is not None for o in out)
    assert all(abs(o - 0.45) < 1e-6 for o in out[20:-10])
    assert out[-1] == 0.46 and g.source == "LIDAR"


def test_disabled_passes_raw():
    g = LidarGate(LidarGateParams(enabled=False))
    assert [g.step(h, i * DT) for i, h in enumerate([0.4, 0.9, 0.4])] == [0.4, 0.9, 0.4]


def test_idle_outside_loiter_guided_and_events():
    from src.inputs.distance_sensor.sensor_state import DistanceState
    st, mode, msgs = DistanceState(), {"on": False}, []
    st.gate.p.enabled = True
    st.set_gate_inputs(vz_up_fn=lambda: 0.0, cam_h_fn=lambda: None,
                       active_fn=lambda: mode["on"], on_event=msgs.append)
    for h in [0.45, 0.95, 0.45]:                 # e.g. baro climb in another mode: raw passes
        st.update_distance(h, "OK")
        assert st.distance_m == h and st.gate.source == "LIDAR_RAW"
    mode["on"] = True                            # LOITER/GUIDED armed: gate acts
    for h in [0.45, 0.45, 0.95, 0.95, 0.45]:
        st.update_distance(h, "OK")
        assert st.distance_m == 0.45, st.distance_m
    assert any("REJECTED" in m for m in msgs) and any("back to normal" in m for m in msgs), msgs
    st.set_gate_inputs(vz_up_fn=lambda: 0.0, cam_h_fn=lambda: 1.077 * 0.45 - 0.025,
                       active_fn=lambda: True, on_event=msgs.append)
    st.update_distance(0.95, "OK")
    assert abs(st.distance_m - 0.45) < 0.002 and st.gate.source == "CAMERA"
    assert "CAMERA corrects" in msgs[-1], msgs[-1]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("PASS", name)
