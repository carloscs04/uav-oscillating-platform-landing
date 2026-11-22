# Experiments

Recordings from tests and flights, one folder per kind and one sub-folder per day.

```
experiments/
  flights/<YYYY-MM-DD>/flight_<HH-MM-SS>_<codename>.csv      ground station, written while ARMED
                       flight_<HH-MM-SS>_<codename>_events.log
  vision/<YYYY-MM-DD>/visionlog_<HH-MM-SS>.csv               onboard Pi, CLI `visionlog [s]`
```

- `flights/` is written by `ground_station/utils/flight_logger.py` on the laptop (MAVLink telemetry, 20 Hz).
- `vision/` is written by the onboard CLI command `visionlog` on the Pi: pixel target, lidar,
  raw and tilt-compensated marker offset, Pixhawk roll/pitch (20 Hz).
