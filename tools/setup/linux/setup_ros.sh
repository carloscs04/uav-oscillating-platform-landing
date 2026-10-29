#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$SCRIPT_DIR/../ros2_ws"
PX4_DIR="$HOME/Tools/PX4-Autopilot"

echo "=== Setting up ROS 2 Workspace ==="

# 1. Clone px4_msgs if not already present
if [ ! -d "$WS_DIR/src/px4_msgs" ]; then
    echo "Cloning px4_msgs..."
    git clone https://github.com/PX4/px4_msgs.git "$WS_DIR/src/px4_msgs"
fi

# 2. Sync message definitions for 96-byte payload match
echo "Syncing PX4 message definitions from PX4-Autopilot..."
rm -f "$WS_DIR/src/px4_msgs/msg/"*.msg
cp "$PX4_DIR/msg/"*.msg "$WS_DIR/src/px4_msgs/msg/"
cp "$PX4_DIR/msg/versioned/"*.msg "$WS_DIR/src/px4_msgs/msg/" 2>/dev/null || true

# 3. Clean old build artifacts and build with colcon
echo "Building workspace with colcon..."
cd "$WS_DIR"
rm -rf build install log
colcon build

echo "=== ROS 2 Setup Complete! ==="
