# Simulation Launch Tool

This directory contains the automated launcher script for starting the complete simulation stack.

---


## `ros_simulation.sh`


### Purpose
Automates the initialization, process cleanup, environment variable sanitization, and execution of all simulation components across separate native Linux terminal windows.

### OS Compatibility
* **Supported OS:** Linux (Ubuntu 22.04 / 24.04 or WSL2 with GUI support).
* **Windows Support:** **Does NOT work on Windows.** Windows developers focus solely on Python development and do not execute this script.

### Key Features & Workflow
1. **Process Cleanup:** Kills any orphaned background instances of Gazebo (`gz sim`), PX4 SITL, or Micro-XRCE-DDS Agent to prevent port and socket conflicts.
2. **Snap Environment Fix:** Sanitizes `XDG_DATA_DIRS`, `GSETTINGS_SCHEMA_DIR`, and `GTK_PATH` variables so `gnome-terminal` can spawn windows properly when executed from inside VS Code (Snap version).
3. **Terminal 1 (Micro-XRCE-DDS Agent):** Spawns a terminal running the DDS agent on UDP port `8888` for PX4-to-ROS 2 communication.
4. **Terminal 2 (PX4 SITL + Gazebo):** Spawns a terminal that navigates to `$PX4_DIR` and executes `make px4_sitl gz_x500`.
5. **Terminal 3 (ROS 2 Workspace):** Spawns a terminal that sources system ROS 2 setup files, compiles `ros2_ws` using `colcon build --symlink-install`, sources `install/setup.bash`, and starts `landing_node`.

### Usage
From the codebase root directory:
```bash
./tools/launch/ros_simulation.sh
```