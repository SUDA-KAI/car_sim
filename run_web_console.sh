#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ../install/local_setup.bash
export ROS_DOMAIN_ID="${SIM_ROS_DOMAIN_ID:-73}"
cd ..
exec /usr/bin/python3 -m gz_sim.web_console.server --domain "$ROS_DOMAIN_ID" "$@"
