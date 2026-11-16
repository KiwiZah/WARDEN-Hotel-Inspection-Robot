#!/usr/bin/env bash
set -euo pipefail
PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then
  source "$HOME/turtlebot3_ws/install/setup.bash"
fi
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
# Required shared config should be provided by systemd EnvironmentFile or shell env.
exec ros2 run joy joy_node
