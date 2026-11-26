"""PID tuning log: only ARMED + GUIDED, duplicates skipped, summary written.
Run: PYTHONPATH=. python3 tests/vision/test_pid_logger.py"""
import csv
import glob
import os
import random
import tempfile

from ground_station.utils.pid_logger import PidTuningLogger

LOITER, GUIDED = 5, 4


def run():
    tmp = tempfile.mkdtemp()
    state = {"v": (True, LOITER)}
    fc = {"v": 0.0}
    log = PidTuningLogger(base_dir=tmp, mode_source=lambda: state["v"],
                          fc_source=lambda: {"fc_fwd": fc["v"], "fc_right": 0.0, "fc_down": 0.0, "fc_h": 0.35})
    t = 100.0

    def pk(f, phase="DESCEND", det=True, sd=0.0):
        return {"pixel_target": {"detected": det, "source": "4X4_50 #10", "h_m": 0.35,
                                 "pad_fwd_m": f, "pad_right_m": 0.0},
                "landing_shadow": {"phase": phase, "err_m": abs(f), "v_fwd": 0.3 * f, "sent_v_fwd": 0.3 * f,
                                   "v_right": 0.0, "v_down": 0.0, "sent_v_down": sd, "note": ""}}

    def feed(p, n=1):
        nonlocal t
        for _ in range(n):
            t += 0.033
            log.process_packets(p, fresh={"pixel_target"}, now=t)

    feed(pk(0.05), 5)                                   # LOITER: nothing
    assert glob.glob(os.path.join(tmp, "*", "*.csv")) == []
    state["v"] = (True, GUIDED)
    hist, rnd = [], random.Random(1)
    for i in range(60):
        hist.append(0.3 * rnd.uniform(-0.05, 0.05))     # non-periodic command
        fc["v"] = 0.8 * hist[max(0, i - 6)]               # Pixhawk: 0.8 x the command ~0.4 s late
        feed(pk(hist[-1] / 0.3 + i * 1e-4, sd=-0.05), 2)   # each frame twice: one row kept
    feed(pk(0.0, "FINAL"), 3)
    state["v"] = (True, LOITER)
    feed(pk(0.0, "FINAL"), 60)                          # > grace: closes
    files = glob.glob(os.path.join(tmp, "*", "*.csv"))
    assert len(files) == 1, files
    rows = list(csv.DictReader(open(files[0])))
    assert 50 <= len(rows) <= 70, len(rows)
    assert rows[0]["id"] == "10" and rows[0]["phase"] == "DESCEND"
    summ = open(files[0][:-4] + "_summary.txt").read()
    assert "DESCEND" in summ and "FINAL" in summ and "asked to climb" in summ, summ
    line = next(x for x in summ.splitlines() if x.startswith("fc follows sent_fwd"))
    assert "delay ~0.4 s" in line and 0.75 <= float(line.split("gain ")[1].split(",")[0]) <= 0.85, line
    print(summ)
    print("PASS pid logger")


if __name__ == "__main__":
    run()
