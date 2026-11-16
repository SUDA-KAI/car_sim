#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export SIM_ROS_DOMAIN_ID="${SIM_ROS_DOMAIN_ID:-43}"
export SIM_IGN_PARTITION="${SIM_IGN_PARTITION:-patrol_sensors}"
# Only sensor-derived state enters this launch; baseline test routes are not sent.
exec ./run_sim.sh "world:=$PWD/test_scenes/worlds/narrow_turns.sdf" \
  "sensor_slam_config:=$PWD/config/sensor_lap_slam.yaml" \
  localization:=sensors controller:=mppi nav:=true patrol:=false teleop:=false \
  max_speed:=0.5 max_yaw_rate:=0.4 record_metrics:=false sensor_monitor_port:=8769 \
  camera_width:=384 camera_rate:=4 "$@"
