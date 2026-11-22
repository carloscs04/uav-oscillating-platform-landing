# Experiments

Recordings from tests and flights, one folder per kind and one sub-folder per day.

```
experiments/
  flights/<YYYY-MM-DD>/flight_<HH-MM-SS>_<codename>.csv      ground station, written while ARMED
                       flight_<HH-MM-SS>_<codename>_events.log
  vision/<YYYY-MM-DD>/visionlog_<HH-MM-SS>.csv               ground station, while the Pi CLI `visionlog [s]` runs
```

- `flights/` is written by `ground_station/utils/flight_logger.py` on the laptop (MAVLink telemetry, 20 Hz).
- `vision/` is also written on the laptop (`ground_station/utils/vision_logger.py`) while the Pi
  CLI command `visionlog` runs: pixel target, lidar, raw and tilt-compensated marker offset,
  Pixhawk roll/pitch (~30 Hz). The Pi only raises a flag; it writes no files.
- Both live only on the ground station: nothing is stored on the Raspberry Pi.
