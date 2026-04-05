#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 base install (adjust distro if not humble)
ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source the local workspace overlay
source "$SCRIPT_DIR/install/setup.bash"

# Build if the executable doesn't exist yet
EXECUTABLE="$SCRIPT_DIR/install/POBSControl/lib/POBSControl/POBSControl"
if [ ! -f "$EXECUTABLE" ]; then
    echo "[run.sh] Executable not found — building first..."
    cd "$SCRIPT_DIR"
    colcon build --packages-select POBSControl
    source "$SCRIPT_DIR/install/setup.bash"
fi

echo "[run.sh] Starting POBSControl..."
cd "$SCRIPT_DIR/POBSControl"
ros2 run POBSControl POBSControl
