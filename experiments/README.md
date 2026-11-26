# Experiments

Recordings from tests and flights, one folder per kind and one sub-folder per day.

```
experiments/
  flights/<YYYY-MM-DD>/flight_<HH-MM-SS>_<codename>.csv      ground station, written while ARMED
                       flight_<HH-MM-SS>_<codename>_events.log
  vision/visionlog/<YYYY-MM-DD>/auto_<HH-MM-SS>.csv         ground station, ARMED in LOITER/GUIDED (full vision log)
  vision/pid/<YYYY-MM-DD>/pid_<HH-MM-SS>.csv                  ground station, only ARMED + GUIDED (control tuning)
                         pid_<HH-MM-SS>_summary.txt
```

- `flights/` is written by `ground_station/utils/flight_logger.py` on the laptop (MAVLink telemetry, 20 Hz).
- `vision/visionlog/` is also written on the laptop (`ground_station/utils/vision_logger.py`) while the Pi
  CLI command `visionlog` runs: pixel target, lidar, raw and tilt-compensated marker offset,
  Pixhawk roll/pitch (~30 Hz). The Pi only raises a flag; it writes no files.
- `vision/pid/` (`ground_station/utils/pid_logger.py`) is the short version for tuning the landing
  law: only the GUIDED part, one row per new frame, error / phase / commanded and sent velocity /
  note. The `_summary.txt` has time, error and swings per phase, marker losses and whether it sank
  while asked to climb. It also has the Pixhawk's measured velocity (fc_*), so the summary
  tells how late and how strongly the drone follows the commands. Send these two files to review a landing.
- All live only on the ground station: nothing is stored on the Raspberry Pi.
