"""Auto vision logging: only while ARMED in LOITER or GUIDED. Run: PYTHONPATH=. python tests/vision/test_vision_logger_modes.py"""
import csv
import glob
import os
import tempfile

from ground_station.utils.vision_logger import VisionExperimentLogger

ALT_HOLD, GUIDED, LOITER = 2, 4, 5


def run():
    tmp = tempfile.mkdtemp()
    state = {"v": None}
    log = VisionExperimentLogger(base_dir=tmp, mode_source=lambda: state["v"])
    pk = {"pixel_target": {"detected": True, "fwd_m": 0.1}, "landing_shadow": {"phase": "ALIGN"}}
    t = 100.0

    def step(v, n=3):
        nonlocal t
        state["v"] = v
        for _ in range(n):
            t += 0.05
            log.process_packets(pk, fresh={"pixel_target"}, now=t)

    step((True, ALT_HOLD))                     # armed but AltHold: nothing
    step((False, LOITER))                      # Loiter but disarmed: nothing
    step(None)                                 # no MAVLink: nothing
    assert glob.glob(os.path.join(tmp, "*", "*.csv")) == []
    step((True, LOITER)); step((True, GUIDED))  # one file across Loiter -> Guided
    step((True, ALT_HOLD), n=2)                 # short flicker: stays open, rows not written
    step((True, GUIDED))
    step((True, ALT_HOLD), n=40)                # 2 s outside: closes
    log.close()
    files = glob.glob(os.path.join(tmp, "*", "*.csv"))
    assert len(files) == 1, files
    rows = list(csv.DictReader(open(files[0])))
    assert len(rows) == 9 and {r["fc_mode"] for r in rows} == {"LOITER", "GUIDED"}
    step((True, LOITER)); log.close()           # new flight segment -> second file
    assert len(glob.glob(os.path.join(tmp, "*", "*.csv"))) == 2
    print("PASS vision logger modes")


if __name__ == "__main__":
    run()
