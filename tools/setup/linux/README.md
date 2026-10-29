# Linux Environment Setup Tools

This directory contains setup scripts to configure Python virtual environments and ROS 2 workspaces on Linux operating systems.

---

## 1. `setup_python.sh`

### Purpose
Initializes a standardized Python virtual environment (`.venv`) for Linux developers and installs the codebase as an editable package.

### What It Does
- Checks if `.venv` exists at the project root; creates it if missing.
- Activates `.venv` and upgrades `pip` to the latest version.
- Installs required Python dependencies listed in `requirements.txt`.
- Executes `pip install -e .` at the codebase root. This mounts the codebase in **editable mode**, allowing imports (e.g., `from precision_landing...`) to work universally across all project subfolders without modifying `sys.path`.

### Usage
```bash
./tools/setup/linux/setup_python.sh
```

---

## 2. `setup_ros.sh`

### Purpose
Initializes, synchronizes, and builds the ROS 2 workspace (`ros2_ws`) to guarantee uORB message definition compatibility between PX4 SITL and ROS 2 nodes.

### What It Does
- **Clones `px4_msgs`:** Checks if `ros2_ws/src/px4_msgs` exists; clones the official repository if missing.
- **Synchronizes Message Definitions:** Copies `.msg` files directly from `$HOME/Tools/PX4-Autopilot/msg/` (and its `versioned/` directory) into `px4_msgs/msg/`. This aligns message struct layouts (such as 96-byte payload matches) between PX4 SITL firmware and ROS 2 topics.
- **Clean Build:** Purges old build artifacts (`build/`, `install/`, `log/`) to eliminate CMake cache conflicts, then compiles the workspace using `colcon build`.

### Usage
```bash
./tools/setup/linux/setup_ros.sh
```

