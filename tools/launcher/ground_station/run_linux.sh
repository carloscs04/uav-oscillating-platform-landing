#!/bin/bash
set -e

# Resolve repository root (3 levels up from tools/launcher/ground_station)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

cd "$ROOT_DIR"

# Default to hardware mode
MODE="real"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --sim)  MODE="sim" ;;
        --real) MODE="real" ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

export PYTHONUNBUFFERED=1
export PX4_DIR="${PX4_DIR:-$HOME/Tools/PX4-Autopilot}"

echo "=================================================="
echo " Starting UAV System on Linux [Mode: ${MODE^^}]"
echo "=================================================="

if [ "$MODE" == "sim" ]; then
    export UAV_MODE="SIMULATION"

    echo "[Setup] Cleaning old SITL processes..."
    pkill -9 -f px4 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f ruby 2>/dev/null || true
    sleep 0.2
else
    export UAV_MODE="HARDWARE"
fi

# Source System ROS 2 & Venv
source /opt/ros/jazzy/setup.bash
if [ -f "$ROOT_DIR/.venv/bin/activate" ]; then
    source "$ROOT_DIR/.venv/bin/activate"
fi

# Build & Source Workspace
cd "$ROOT_DIR/ros2_ws"
colcon build --symlink-install
source "$ROOT_DIR/ros2_ws/install/setup.bash"

export PYTHONPATH="$ROOT_DIR:$PYTHONPATH"

# Launch ROS 2 bringup (UAV_MODE environment variable is inherited globally)
ros2 launch uav_opl_bringup system.launch.py