# **UAV Oscillating Platform**
>**A vision-guided quadcopter that tracks an oscillating platform and syncs its descent for a soft, precise landing.**

## Members

| Name | Student ID |
|---|---|
| Carlos Cuadras Santaella | A01747209 |
| Ana Patricia González Alcocer | A01285361 |
| Aleany Lizeth Pérez Chávez | A00836152 |

## Overview

This project presents a **vision-guided autonomous landing system** engineered to achieve soft touchdowns on dynamic platforms subject to manual or automated surface perturbations. Mounted on an **F450 quadcopter** powered by a **Pixhawk flight controller**, a downward-facing webcam feeds real-time **ArUco marker pose estimation** into a processing system. This architecture executes a phase-tracking algorithm to analyze multi-axis (x, y, z) platform motion, accommodating both sinusoidal and irregular non-sinusoidal oscillations during a brief tracking hover window. Once synchronized, the system transmits commands to the flight controller. By matching the quadcopter's descent rate with the platform's relative motion, the controller minimizes contact velocity to execute a gentle, precise touchdown before disarming. 

## Components

### Hardware

| Component | Role |
|---|---|
| F450 quadcopter frame | Airframe |
| Pixhawk flight controller (PX4) | Low-level flight control; receives velocity setpoints over MAVLink |
| Raspberry Pi 5 | Companion computer; runs vision, tracking and control |
| USB webcam (downward-facing) | Captures the ArUco marker on the platform |
| Distance sensor (rangefinder) | Measures height above the platform |
| Oscillating platform with ArUco marker | Landing target |

### Software

| Tool | Version | Used for |
|---|---|---|
| Python | 3.12 | All project code |
| ROS 2 | Jazzy (Ubuntu 24.04) | Simulation |
| PX4-Autopilot | SITL | Simulated flight controller |
| Gazebo | Harmonic | Physics simulation (`gz_x500` model) |
| pymavlink | ≥ 2.4.40 | Communication with PX4 |
| OpenCV / NumPy | < 4.12 / < 2 | Image processing and ArUco detection (pinned for ROS `cv_bridge` compatibility) |

## Repository Architecture

The codebase is split into two layers:

- **`src/` (pure Python):** the algorithms and drivers, with no ROS imports outside `interfaces/software/`. Can be developed and tested on any OS, including Windows.
- **`ros2_ws/` (ROS 2 wrapper):** thin ROS 2 nodes that import code from `src/` and connect it through topics.

```
uav-oscillating-platform-landing/
├── src/                              # Pure Python core (no ROS)
│   ├── main.py                       # Standalone test loop without ROS
│   ├── core/drone_state.py           # Telemetry data container
│   ├── modules/
│   │   ├── vision/tracker.py         # Target detection
│   │   └── control/pid_controller.py # Platform tracking / descent controller
│   └── interfaces/
│       ├── hardware/mavlink/         # MAVLink driver, manager and config.yaml
│       └── software/ros2/            # ROS 2 bridge helpers
├── ros2_ws/src/                      # ROS 2 packages (ament_python)
│   ├── px4_mavlink/                  # px4_node: MAVLink telemetry → /uav/state
│   ├── sensors/camera_model_usb/     # camera_node: webcam → /camera/image_raw
│   ├── sensors/distance_sensor/      # rangefinder_node: → /sensor/distance
│   ├── aruco_perception/             # aruco_node: image → /target/pose
│   ├── uav_opl_control/              # control_node: landing controller
│   └── uav_opl_bringup/              # Launch files (system.launch.py)
├── launch/                           # run_linux.sh (sim / real), run_windows.bat
├── tests/SIL/                        # Software-in-the-loop tests against PX4 SITL
├── tools/setup/                      # Python environment setup (linux/, windows/)
├── requirements.txt
└── setup.py                          # Installs the repo in editable mode
```

### `src/` Structure

| Folder | What goes here |
|---|---|
| `core/` | Shared data types (e.g. `DroneState`). No logic. |
| `modules/` | Algorithms: vision, tracking, control. No hardware access. |
| `interfaces/hardware/` | Device drivers: Pixhawk (MAVLink), camera, rangefinder. |
| `interfaces/software/` | Adapters to other software (ROS 2). |
| `main.py` | Runs the pipeline without ROS. |

## ROS Simulation

The simulation runs on **Ubuntu 24.04** with ROS 2 Jazzy, PX4 SITL and Gazebo Harmonic.

## Physical Integration


| Connection | Port | Settings |
|---|---|---|
| Raspberry Pi ↔ Pixhawk (MAVLink) | CP2102 USB adapter on TELEM 2 (`/dev/serial/by-id/usb-Silicon_Labs_CP2102_...`) | 921600 baud |
| Rangefinder → Raspberry Pi | `/dev/ttyUSB0` | Set with the `port` parameter |
| Webcam → Raspberry Pi | USB, device index `0` | Set with the `device_id` parameter |
