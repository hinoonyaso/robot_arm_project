#!/usr/bin/env bash
set -euo pipefail

# Source ROS2 and workspace
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

# Build if needed
if [ ! -d install ]; then
  echo "[run_demo] Building workspace..."
  colcon build --symlink-install
  source install/setup.bash
fi

# Launch bringup
echo "[run_demo] Starting demo..."
ros2 launch pick_place_bringup demo.launch.py
