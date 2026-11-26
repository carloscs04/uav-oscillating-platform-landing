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
    assert law.step((0.03, 0.0), 0.25, 0.5)["v_down"] == 0.0    # 3 cm: still centring
    assert not law.committed
    o = law.step((0.02, 0.0), 0.25, 0.6)
    assert o["v_down"] == 0.0 and "centring" in o["note"]
    assert law.step((0.02, 0.0), 0.25, 1.0)["v_down"] == 0.0    # 0.4 s centred: not yet
    assert law.step((0.02, 0.0), 0.25, 1.15)["v_down"] > 0.0    # 0.55 s centred: commit
    assert law.committed


def test_strict_no_blind_descent_when_lost_off_centre():
    """Flight 2026-11-24 21:53: drifted to 4.3 cm and lost the marker at 0.22 m."""
    law = LandingLaw(LandingParams(align_hold_s=0.0))
    law.strict_descent = True
    law.phase, law.committed = "FINAL", True
    law.step((0.043, 0.0), 0.22, 0.0)
    o = law.step(None, 0.22, 0.05)
    assert o["v_down"] == 0.0 and not law.committed            # brake, no blind descent
    for i in range(80):                                         # then climbs to search again
        o = law.step(None, 0.22, 0.1 + 0.05 * i)
    assert o["v_down"] < 0
    law2 = LandingLaw(LandingParams(align_hold_s=0.0))          # lost centred: blind descent ok
    law2.strict_descent = True
    law2.phase, law2.committed = "FINAL", True
    law2.step((0.02, 0.0), 0.22, 0.0)
    assert law2.step(None, 0.22, 0.05)["v_down"] > 0


def test_params_from_config():
    p = LandingParams.from_dict({"kp": 0.8, "v_max": 0.2, "unknown_key": 5})
    assert p.kp == 0.8 and p.v_max == 0.2


def test_final_lifted_back_above_goes_descend():
    """Flight 2026-11-25 20:18: reacquire climb left it at 0.40 m in FINAL, centred but never descending."""
    law = LandingLaw(LandingParams(align_hold_s=0.0))
    law.strict_descent = True
    law.step((0.0, 0.0), 0.6, 0.0)
    law.step((0.0, 0.0), 0.6, 0.1)
    assert law.step((0.0, 0.0), 0.28, 0.2)["phase"] == "FINAL"
    assert law.step((0.0, 0.0), 0.33, 0.3)["phase"] == "FINAL"   # small bump: stays
    out = law.step((0.0, 0.0), 0.40, 0.4)
    assert out["phase"] == "DESCEND"
    for i in range(30):                                          # still, centred: comes back down
        out = law.step((0.0, 0.0), 0.40, 0.5 + 0.1 * i)
    assert out["phase"] == "DESCEND" and out["v_down"] > 0


def test_final_corrects_small_errors():
    """Flight 2026-11-25 20:28: the 2 cm deadband let a 0.3 -> 3.4 cm drift grow unchecked low down."""
    law = LandingLaw(LandingParams(align_hold_s=0.0))
    out = law._horizontal(0.015, 0.0, low=True)
    assert out[0] > 0                                            # 1.5 cm in FINAL: P pulls back
    assert law._horizontal(0.015, 0.0, low=False) == (0.0, 0.0)  # higher up: deadband as before


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


def test_final_waits_for_3cm_even_committed():
    law, t = LandingLaw(), 0.0
    for _ in range(60):                                  # align, then descend into FINAL
        law.step((0.0, 0.0), 0.35, t); t += 0.05
    o = law.step((0.0, 0.0), 0.25, t); t += 0.05         # below commit_h, centred -> committed
    assert o["phase"] == "FINAL" and law.committed and o["v_down"] > 0
    o = law.step((0.04, 0.0), 0.24, t); t += 0.05        # 4 cm off: wait, with a message
    assert o["v_down"] == 0.0 and "4.0 cm > 3 cm" in o["note"]
    o = law.step((0.025, 0.0), 0.24, t); t += 0.05       # back inside 3 cm -> descends again
    assert o["v_down"] > 0
    o = law.step((0.07, 0.0), 0.14, t)                   # very low: keeps going down
    assert o["v_down"] > 0


def test_lost_off_centre_but_on_ground_touches_down_not_climbs():
    """Flight 2026-11-24 22:27: lost #20 3.6 cm off-centre, sank onto the pad (lidar 0.09),
    then climbed off the ground after lost_search_s and went unstable."""
    law = LandingLaw(LandingParams(align_hold_s=0.0))
    law.strict_descent = True
    law.phase, law.committed = "FINAL", True
    law.step((0.036, 0.0), 0.16, 0.0)
    o = law.step(None, 0.16, 0.05)                         # still in the air: no blind descent
    assert o["v_down"] == 0.0
    for i in range(100):                                   # 5 s on the ground: never climbs
        o = law.step(None, 0.09, 0.1 + 0.05 * i)
        assert o["v_down"] >= 0.0, (i, o)
    assert law.phase == "TOUCHDOWN"


def test_touchdown_stage_slows_before_final():
    """Flight 2026-11-25 19:22: TOUCHDOWN from 0.39 m reached FINAL at 15 cm/s and sank to 0.17."""
    law = LandingLaw(LandingParams(align_hold_s=0.0))
    law.strict_descent = True
    law.step((0.0, 0.0), 0.6, 0.0)
    assert law.step((0.0, 0.0), 0.6, 0.1)["v_down"] == law.p.descend_speed
    assert law.step((0.0, 0.0), 0.38, 0.2)["v_down"] == law.p.final_speed
    law2 = LandingLaw(LandingParams(align_hold_s=0.0))       # APPROACH/normal: unchanged
    law2.step((0.0, 0.0), 0.38, 0.0)
    assert law2.step((0.0, 0.0), 0.38, 0.1)["v_down"] == law2.p.descend_speed


def test_descend_waits_until_still_before_final():
    """Flight 2026-11-25 19:38: reached 0.30 m crossing the centre at 8 cm/s and overshot."""
    law = LandingLaw(LandingParams(align_hold_s=0.0))
    law.step((0.0, 0.0), 0.6, 0.0)
    law.step((0.0, 0.0), 0.6, 0.1)                              # DESCEND
    law.step((-0.02, 0.0), 0.36, 0.2)
    out = law.step((0.02, 0.0), 0.36, 0.3)                      # 40 cm/s raw: still moving
    assert out["phase"] == "DESCEND" and out["v_down"] == 0.0 and "settling" in out["note"]
    for i in range(30):                                         # still: descends again
        out = law.step((0.0, 0.0), 0.36, 0.4 + 0.1 * i)
    assert out["v_down"] > 0


def test_lost_low_climbs_early_to_see_it():
    """Lost the marker at 0.28 m: climb after lost_hold_s instead of braking 3 s."""
    law = LandingLaw(LandingParams(align_hold_s=0.0))
    law.strict_descent = True                                   # TOUCHDOWN stage, not committed
    law.step((0.0, 0.0), 0.6, 0.0)
    law.step((0.0, 0.0), 0.6, 0.1)
    law.step((0.0, 0.0), 0.28, 0.2)                             # FINAL
    assert law.step(None, 0.28, 0.5)["v_down"] == 0.0           # brake first
    out = law.step(None, 0.30, 0.9)
    assert out["phase"] == "FINAL" and out["v_down"] == -law.p.reacquire_speed
    assert law.step(None, 0.45, 1.5)["v_down"] == 0.0           # high enough: stop climbing
