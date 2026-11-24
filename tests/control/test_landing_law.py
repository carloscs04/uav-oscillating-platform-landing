"""
test_landing_law.py - landing law phases and signs (no hardware).

    python3 tests/control/test_landing_law.py
    python3 -m pytest tests/control/test_landing_law.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.logic.control.landing_law import LandingLaw, LandingParams  # noqa: E402


def test_signs_and_limit():
    law = LandingLaw(LandingParams(kd=0.0))             # P only: these jumps are not real motion
    o = law.step((0.10, 0.0), 1.0, 0.0)          # target 10 cm in FRONT -> go forward
    assert o["phase"] == "ALIGN" and o["v_fwd"] > 0 and o["v_right"] == 0 and o["v_down"] == 0
    o = law.step((0.0, -0.10), 1.0, 0.1)         # target to the LEFT -> negative right
    assert o["v_right"] < 0
    o = law.step((2.0, 2.0), 1.0, 0.2)           # far away -> limited to v_max
    assert abs((o["v_fwd"] ** 2 + o["v_right"] ** 2) ** 0.5 - 0.30) < 1e-3
    o = law.step((0.01, 0.0), 1.0, 0.3)          # inside the deadband -> no command
    assert o["v_fwd"] == 0 and o["v_right"] == 0


def test_full_descent():
    law, t, h = LandingLaw(), 0.0, 1.0
    phases = []
    while t < 60 and law.phase != "TOUCHDOWN":
        o = law.step((0.01, -0.01), h, t)
        h = max(0.0, h - o["v_down"] * 0.05)
        phases.append(o["phase"])
        t += 0.05
    assert law.phase == "TOUCHDOWN", (law.phase, h)
    order = [p for i, p in enumerate(phases) if i == 0 or p != phases[i - 1]]
    assert order == ["ALIGN", "DESCEND", "FINAL", "TOUCHDOWN"], order
    assert phases.count("ALIGN") * 0.05 >= 2.0          # waited align_hold_s before descending


def test_pause_when_error_grows():
    law = LandingLaw()
    for i in range(50):
        law.step((0.0, 0.0), 1.0, i * 0.05)
    assert law.phase == "DESCEND"
    o = law.step((0.15, 0.0), 0.9, 2.6)                # 15 cm off -> stop descending, re-align
    assert o["v_down"] == 0 and o["v_fwd"] > 0


def test_lost_target():
    law = LandingLaw()
    for i in range(50):
        law.step((0.0, 0.0), 0.5, i * 0.05)
    assert law.phase == "DESCEND"
    o = law.step(None, 0.5, 2.7)                        # just lost -> brake, keep phase
    assert o["phase"] == "DESCEND" and o["v_down"] == 0 and o["v_fwd"] == 0
    o = law.step(None, 0.5, 6.0)                        # lost > 3 s low -> SEARCH and climb
    assert o["phase"] == "SEARCH" and o["v_down"] < 0
    o = law.step(None, 0.9, 7.0)                        # high enough -> stop climbing
    assert o["v_down"] == 0
    o = law.step((0.0, 0.0), 0.9, 7.1)                  # found again -> ALIGN
    assert o["phase"] == "ALIGN"


def test_align_timeout_and_rearm():
    law = LandingLaw(LandingParams(align_timeout_s=1.0))
    for i in range(30):
        o = law.step((0.2, 0.0), 1.0, i * 0.05)          # never aligned
    assert o["phase"] == "HOLD" and o["v_fwd"] == 0
    o = law.step((0.0, 0.0), 1.1, 2.0)
    assert o["phase"] == "HOLD"                          # stays until lifted / reset
    o = law.step((0.0, 0.0), 1.4, 2.1)                   # lifted 30+ cm -> again
    assert o["phase"] == "ALIGN"
    law.reset()
    assert law.phase == "SEARCH"


def test_hold_rearms_when_target_gone():
    law = LandingLaw(LandingParams(align_timeout_s=1.0))
    for i in range(30):
        o = law.step((0.2, 0.0), 1.0, i * 0.05)
    assert o["phase"] == "HOLD"
    o = law.step(None, 1.0, 2.0)                         # gone 0.6 s -> still HOLD
    assert o["phase"] == "HOLD"
    o = law.step(None, 1.0, 5.0)                         # gone > 3 s -> re-armed
    assert o["phase"] == "SEARCH"
    o = law.step((0.0, 0.0), 1.0, 5.1)
    assert o["phase"] == "ALIGN"


def test_d_term_opposes_drift():
    law = LandingLaw(LandingParams(kp=0.0, deadband=0.0))
    o = None
    for i in range(20):                                  # target sliding right at 0.1 m/s
        o = law.step((0.0, 0.05 + 0.1 * i * 0.033), 1.0, i * 0.033)
    assert o["v_right"] > 0.02                            # chases it (drone drifting left)
    law.step(None, 1.0, 1.0)                              # lost -> rate cleared
    o = law.step((0.0, 0.0), 1.0, 1.05)
    assert o["v_right"] == 0


def test_committed_blind_descent_and_touchdown_push():
    law, t, h = LandingLaw(), 0.0, 0.5
    while law.phase != "FINAL":
        o = law.step((0.0, 0.0), h, t)
        h -= o["v_down"] * 0.05
        t += 0.05
    while h > 0.15:
        o = law.step((0.0, 0.0), h, t)
        h -= o["v_down"] * 0.05
        t += 0.05
    assert law.committed
    o = law.step((0.15, 0.0), h, t)                       # big error low: no pause any more
    assert o["v_down"] > 0
    for i in range(100):                                  # pad out of view: keep going down
        o = law.step(None, h, t + 0.05 * (i + 1))
        h = max(0.05, h - o["v_down"] * 0.05)
    assert o["v_down"] > 0 and law.phase == "TOUCHDOWN" and o["v_fwd"] == 0


def test_lost_at_22cm_keeps_descending():
    law, t, h = LandingLaw(), 0.0, 0.5                    # flight 2026-11-22: lost at 0.22 m
    while h > 0.22:
        o = law.step((0.0, 0.0), h, t)
        h -= o["v_down"] * 0.05
        t += 0.05
    assert law.phase == "FINAL" and law.committed
    for i in range(100):                                  # never brakes or climbs
        o = law.step(None, h, t + 0.05 * i)
        assert o["v_down"] > 0, (i, o)
        h = max(0.05, h - o["v_down"] * 0.05)
    assert law.phase == "TOUCHDOWN"


def test_strict_descent_needs_5cm():
    law = LandingLaw(LandingParams(align_hold_s=0.0))
    law.strict_descent = True
    law.step((0.0, 0.0), 0.6, 0.0)
    law.step((0.0, 0.0), 0.6, 0.1)
    assert law.phase == "DESCEND"
    assert law.step((0.07, 0.0), 0.6, 0.2)["v_down"] == 0.0     # 7 cm: holds height
    assert law.step((0.03, 0.0), 0.6, 0.3)["v_down"] > 0.0
    assert law.step((0.07, 0.0), 0.25, 0.4)["v_down"] == 0.0    # low, 7 cm: no commit
    assert not law.committed
    assert law.step((0.03, 0.0), 0.25, 0.5)["v_down"] > 0.0
    assert law.committed


def test_params_from_config():
    p = LandingParams.from_dict({"kp": 0.8, "v_max": 0.2, "unknown_key": 5})
    assert p.kp == 0.8 and p.v_max == 0.2


if __name__ == "__main__":
    failed = 0
    for name, fn in [(k, v) for k, v in dict(globals()).items() if k.startswith("test_")]:
        try:
            fn()
            print("PASS", name)
        except Exception as e:  # noqa: BLE001
            failed += 1
            print("FAIL", name, repr(e))
    print("\nPASS" if not failed else f"\nFAIL ({failed})")
    sys.exit(1 if failed else 0)
