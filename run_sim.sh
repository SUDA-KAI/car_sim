#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
if [[ -f ../install/local_setup.bash ]]; then
  source ../install/local_setup.bash
fi
# Keep simulation velocity commands isolated from the physical chassis.
export ROS_DOMAIN_ID="${SIM_ROS_DOMAIN_ID:-42}"
export IGN_PARTITION="${SIM_IGN_PARTITION:-patrol_sim}"
exec ros2 launch "$PWD/launch/patrol_sim.launch.py" "$@"
