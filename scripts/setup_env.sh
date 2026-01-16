#!/usr/bin/env bash
set -euo pipefail

# Source ROS2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# Install dependencies (user can uncomment as needed)
# rosdep update
# rosdep install --from-paths src --ignore-src -r -y

echo "Environment setup complete."
