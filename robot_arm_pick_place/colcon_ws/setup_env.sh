#!/usr/bin/env bash
set -e

if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f "$(pwd)/install/setup.bash" ]; then
  source "$(pwd)/install/setup.bash"
else
  echo "[setup_env] install/setup.bash not found. Run colcon build first." >&2
fi
