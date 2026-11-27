"""Precision-landing log: only ARMED + LAND, duplicates skipped, summary written.
Run: PYTHONPATH=. python3 tests/vision/test_plnd_logger.py"""
import csv
import glob
import os
import tempfile

from ground_station.utils.plnd_logger import PrecLandLogger

LOITER, LAND = 5, 9


def run():
    tmp = tempfile.mkdtemp()
    state = {"v": (True, LOITER)}
    fc = {"h": 1.2}
    log = PrecLandLogger(base_dir=tmp, mode_source=lambda: state["v"],
                         fc_source=lambda: {"fc_fwd": 0.03, "fc_right": 0.0, "fc_down": 0.1, "fc_h": fc["h"]})
    t = 100.0

    def feed(h, e, det=True, n=1):
        nonlocal t
        for i in range(n):
            t += 0.033
            p = {"pixel_target": {"detected": det, "source": "4X4_50 #10", "h_m": h,
                                  "pad_fwd_m": e + 0.001 * i, "pad_right_m": 0.01,
                                  "lt_x": e, "lt_y": 0.01, "lt_z": h}}
            log.process_packets(p, fresh={"pixel_target"}, now=t)

    feed(1.2, 0.05, n=5)                                  # LOITER: nothing
    assert glob.glob(os.path.join(tmp, "*", "*.csv")) == []
    state["v"] = (True, LAND)
    feed(1.2, 0.06, n=60)                                 # holding while centring
    for k in range(40):                                   # descend
        fc["h"] = 1.2 - 0.028 * (k + 1)
        feed(fc["h"], 0.02)
    log.process_packets({}, fresh=set(), now=t + 0.1)
    p = {"pixel_target": {"detected": True, "source": "4X4_50 #10", "h_m": 0.1,
                          "pad_fwd_m": 0.02, "pad_right_m": 0.01}}
    log.process_packets(p, fresh={"pixel_target"}, now=t + 0.2)    # same frame twice: one row
    log.process_packets(p, fresh={"pixel_target"}, now=t + 0.25)
    state["v"] = (False, LAND)
    log.process_packets({}, fresh=set(), now=t + 1.0)
    log.process_packets({}, fresh=set(), now=t + 3.0)     # grace over: closed
    files = glob.glob(os.path.join(tmp, "*", "flight_*_plnd.csv"))
    assert len(files) == 1, files
    rows = list(csv.DictReader(open(files[0])))
    assert len(rows) == 101, len(rows)
    s = open(files[0][:-4] + "_summary.txt").read()
    print(s)
    assert "held" in s and "last seen" in s and "touchdown" in s
    print("OK")


if __name__ == "__main__":
    run()
