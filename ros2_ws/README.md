# ROS 2 Workspace Operational Guide

This document contains the terminal setup and execution instructions for running the PX4 SITL, Gazebo Harmonic simulation, Micro-XRCE-DDS bridge, and custom ROS 2 nodes.

> **Automated Execution:** You can automatically launch the entire simulation stack (Micro-XRCE-DDS Agent, PX4 SITL + Gazebo, and ROS 2 nodes) using the provided launch script from the repository root:
> ```bash
> ./tools/launch/ros_simulation.sh
> ```

---

## 1. Multi-Terminal Execution Workflow

To run the full simulation stack, open **3 separate terminal windows**.

### Terminal 1: Micro-XRCE-DDS Bridge Agent
Start the DDS agent to translate messages between PX4 uORB topics and ROS 2:
```bash
MicroXRCEAgent udp4 --port 8888
```

### Terminal 2: PX4 SITL + Gazebo
Launch PX4 Software-In-The-Loop with the standard quadcopter model:
```bash
cd ~/path/PX4-Autopilot
make px4_sitl gz_x500
```

### Terminal 3: Precision Landing ROS 2 Node
Source the compiled workspace and run your landing controller node:
```bash
cd ~/uav-oscillating-platform-landing/ros2_ws
source install/setup.bash
ros2 run precision_landing landing_node
```

---

## 2. Telemetry Verification Commands

Run these inside Terminal 3 (after sourcing `install/setup.bash`) to verify hardware data streams:

* **List available bridged topics:**
  ```bash
  ros2 topic list
  ```

* **Inspect live vehicle status telemetry:**
  ```bash
  ros2 topic echo /fmu/out/vehicle_status_v5
  ```

* **Inspect vehicle local position:**
  ```bash
  ros2 topic echo /fmu/out/vehicle_local_position
  ```
