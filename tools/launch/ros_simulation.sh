#!/bin/bash

# Dynamically resolve project directory paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/../.." && pwd )"
WS_DIR="$ROOT_DIR/ros2_ws"
PX4_DIR="${PX4_DIR:-$HOME/Tools/PX4-Autopilot}"

echo "=== Cleaning background Gazebo & PX4 processes ==="
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "px4" 2>/dev/null

sleep 1

echo "=== Launching Precision Landing Simulation Stack ==="

launch_gnome() {
    local title="$1"
    local cmd="$2"

    env XDG_DATA_DIRS="/usr/share/ubuntu:/usr/local/share:/usr/share" \
        GSETTINGS_SCHEMA_DIR="" \
        GTK_PATH="" \
    gnome-terminal --title="$title" -- bash -c "$cmd; exec bash" &
}

# 1. Terminal 1: Micro-XRCE-DDS Agent
launch_gnome "1. Micro-XRCE-DDS Agent" \
"echo '[1/3] Starting Micro-XRCE-DDS Agent...'; MicroXRCEAgent udp4 --port 8888"

# 2. Terminal 2: PX4 SITL + Gazebo
launch_gnome "2. PX4 SITL Gazebo" \
"echo '[2/3] Starting PX4 SITL & Gazebo...'; cd '$PX4_DIR' && make px4_sitl gz_x500"

# Allow PX4 SITL and Gazebo server to initialize
sleep 6

# 3. Terminal 3: ROS 2 Precision Landing Node
launch_gnome "3. ROS 2 Landing Node" \
"echo '[3/3] Sourcing ROS 2, building workspace & launching node...'; \
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; \
elif [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; \
elif [ -f ~/.bashrc ]; then source ~/.bashrc; fi; \
cd '$WS_DIR' && colcon build --symlink-install && source install/setup.bash && ros2 run precision_landing landing_node"