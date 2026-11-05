# Camera testing and calibration

Run these in order from `~/uav-opl` with `.venv/bin/python`. Only one script can
use the camera at a time. All settings live in `config/camera.yaml`.

| Step | Script | What to do | Pass when |
|---|---|---|---|
| 1 | `check_setup.py` | Run it | Every check passes |
| 2 | `live_view.py` | Turn the lens ring, then lock it | Focus score peaks and the marker is detected |
| 3 | `capture_test.py` | Move the marker during a run, then save the values in `camera.yaml` | Detection >= 90% under cage lighting |
| 4 | Not built yet | Capture 20-30 ChArUco board views | Views cover the whole frame |
| 5 | Not built yet | Solve the lens calibration | Low reprojection error |
| 6 | Not built yet | Check pose at taped distances | Error well within the 10 cm landing limit |
| 7 | `benchmark.py` | Run with the marker moving | >= 15 FPS and >= 90% detection |

## After a change

| What changed | Rerun from |
|---|---|
| Lighting, exposure, gain or tuning file | Step 3, then 7 |
| Marker size or ID | Step 6, then 7 |
| Focus, lens or sensor mode | Step 2, then everything after it |
| Camera remounted, same focus | Step 6, then 7 |
