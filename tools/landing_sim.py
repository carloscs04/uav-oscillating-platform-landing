"""Offline landing sim: LandingLaw + a Pixhawk that answers velocity commands `tau` s late at `gain`,
plus its own wobble (stronger below 0.55 m) and an optional moving pad. Not flight truth; use it to
compare settings.  python3 -m tools.landing_sim"""
import math
import random
import sys

from src.logic.control.landing_law import LandingLaw, LandingParams


def run(seed, wobble=0.03, tau=0.9, gain=0.95, pad_v=0.0, cap=0.3, T=120.0, slide=False, **kw):
    random.seed(seed)
    law = LandingLaw(LandingParams(**kw))
    law.strict_descent, law.align_timeout_enabled = True, False
    dt = 0.033
    n = max(1, int(tau / dt))
    bx, by = [0.0] * n, [0.0] * n
    x, y, h, t, retries, last = 0.05, -0.03, 0.9, 0.0, 0, ""
    px = py = 0.0                                   # pad position
    o = {"phase": ""}
    while t < T:
        amp = wobble if h < 0.55 else 0.015
        wx = amp * 2.5 * math.cos(2.5 * t + seed) + 0.02 * random.gauss(0, 1)
        wy = amp * 2.2 * math.sin(2.2 * t + 1 + seed) + 0.02 * random.gauss(0, 1)
        if slide:                               # straight slide, direction changes every 15 s
            ang = seed + 2.0 * int(t // 15)
            px += pad_v * math.cos(ang) * dt
            py += pad_v * math.sin(ang) * dt
        else:                                   # slow circle (always turning: the hard case)
            px += pad_v * math.cos(0.3 * t) * dt
            py += pad_v * math.sin(0.3 * t) * dt
        o = law.step((px - x, py - y), h, t)
        k = o["note"].split(":")[0]
        if k != last and "retry" in o["note"]:
            retries += 1
        last = k
        bx.append(o["v_fwd"]); by.append(o["v_right"])
        x += (gain * bx.pop(0) + wx) * dt
        y += (gain * by.pop(0) + wy) * dt
        h -= max(-0.15, min(cap, o["v_down"])) * dt
        t += dt
        if o["phase"] == "TOUCHDOWN":
            break
    return t, math.hypot(x - px, y - py) * 100, o["phase"], retries


def sim(label="", n=30, **kw):
    res = [run(s, **kw) for s in range(n)]
    ok = [r for r in res if r[2] == "TOUCHDOWN"]
    e = sorted(r[1] for r in ok)
    tt = sorted(r[0] for r in ok)
    s = f"{len(ok)}/{n} landed"
    if ok:
        s += f", t med {tt[len(tt) // 2]:.0f} s, err med {e[len(e) // 2]:.1f} p90 {e[int(len(e) * .9)]:.1f} cm"
    print(f"{label:38s} {s}, retries {sum(r[3] for r in res)}")


if __name__ == "__main__":
    for env in (dict(wobble=0.03), dict(wobble=0.05), dict(wobble=0.03, tau=1.1, gain=0.6),
                dict(wobble=0.03, pad_v=0.03)):
        for kw in (dict(smith_gain=0.0), dict(smith_gain=0.8), dict(smith_gain=0.8, kp_final=0.45)):
            sim(f"{env} {kw}", **env, **kw)
