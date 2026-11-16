#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ../install/local_setup.bash
export ROS_DOMAIN_ID=42
exec /usr/bin/python3 "$PWD/scripts/sensor_monitor.py" "$@" --ros-args -p use_sim_time:=true
