#!/bin/bash
set -e

# Get root directory (1 level up from launch/)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

MODE="sim"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --sim)  MODE="sim" ;;
        --real) MODE="real" ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

echo "=================================================="
echo " Starting UAV System on Linux [Mode: ${MODE^^}]"
echo "=================================================="

# 1. Clean up stale SITL processes
echo "[Setup] Cleaning old background SITL processes..."
pkill -9 -f px4 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
sleep 1

# 2. Set PX4 directory path
export PX4_DIR="${PX4_DIR:-$HOME/Tools/PX4-Autopilot}"

# Helper function to launch gnome-terminal without GTK/GSettings environment conflicts
launch_gnome() {
    local title="$1"
    local cmd="$2"

    env XDG_DATA_DIRS="/usr/share/ubuntu:/usr/local/share:/usr/share" \
        GSETTINGS_SCHEMA_DIR="" \
        GTK_PATH="" \
    gnome-terminal --title="$title" -- bash -c "$cmd; exec bash" &
}

# 3. Handle Simulation Mode Execution
if [ "$MODE" == "sim" ]; then
    export UAV_MODE="SIMULATION"
    USE_SIM="true"

    echo "[Setup] Launching Gazebo SITL in a new terminal window..."
    launch_gnome "PX4 SITL Gazebo" "cd '$PX4_DIR' && make px4_sitl gz_x500"

    echo "[Setup] Waiting for PX4 SITL initialization (8s)..."
    sleep 8
else
    export UAV_MODE="HARDWARE"
    USE_SIM="false"
fi

# 4. Source System ROS 2 & Project Venv
source /opt/ros/jazzy/setup.bash
if [ -f "$ROOT_DIR/.venv/bin/activate" ]; then
    source "$ROOT_DIR/.venv/bin/activate"
fi

# 5. Build Workspace & Source
cd "$ROOT_DIR/ros2_ws"
colcon build --symlink-install
source "$ROOT_DIR/ros2_ws/install/setup.bash"

export PYTHONPATH="$ROOT_DIR:$PYTHONPATH"

# 6. Launch ROS 2 Nodes
ros2 launch uav_opl_bringup system.launch.py use_sim:=$USE_SIM