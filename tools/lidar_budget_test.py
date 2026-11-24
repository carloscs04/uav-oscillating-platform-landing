"""Compare VL53L1X readings at several timing budgets (run with the uav-opl service STOPPED)."""
import statistics, sys, time
sys.path.insert(0, "/home/pi/uav-opl")
from src.inputs.distance_sensor.vl53l1x import VL53L1X, STATUS_TEXT

for budget in (33, 50, 100, 200):
    with VL53L1X(distance_mode="long", timing_budget_ms=budget) as tof:
        tof.start_ranging()
        ok, bad = [], {}
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            mm = tof.read_mm()
            if mm is None:
                k = STATUS_TEXT.get(tof.last_status, tof.last_status)
                bad[k] = bad.get(k, 0) + 1
            else:
                ok.append(mm)
        med = f"{statistics.median(ok):5.0f} mm (min {min(ok)}, max {max(ok)})" if ok else "  no valid reading"
        print(f"budget {budget:3d} ms: {len(ok):3d} valid, median {med}  invalid: {bad or '-'}")
