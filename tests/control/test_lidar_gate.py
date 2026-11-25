"""
test_lidar_gate.py - lidar spike gate on synthetic and recorded heights (no hardware).

    python3 tests/control/test_lidar_gate.py
"""

import csv
import glob
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.logic.control.lidar_gate import LidarGate, LidarGateParams  # noqa: E402

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
    """Every armed flight from the platform day (2026-11-23): no jumps there -> nothing rejected."""
    root = Path(__file__).resolve().parents[2] / "experiments" / "flights" / "2026-11-23"
    files = sorted(glob.glob(str(root / "*.csv")))
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
            g.step(h, t, -vz, None)
            total += 1
        rej += g.rejected_total
    print(f"  recorded 2026-11-23: {total} samples, {rej} rejected")
    assert rej <= total * 0.001


def test_disabled_passes_raw():
    g = LidarGate(LidarGateParams(enabled=False))
    assert [g.step(h, i * DT) for i, h in enumerate([0.4, 0.9, 0.4])] == [0.4, 0.9, 0.4]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("PASS", name)
