#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# Fixed occupancy map + AMCL + Hybrid A* + MPPI. Camera perception is a later
# task; navigation and obstacle costs currently use lidar and sonar.
exec ./run_sim.sh \
  "world:=$PWD/worlds/map_20260817_154310_filtered.sdf" \
  localization:=amcl controller:=mppi nav:=true patrol:=false teleop:=false \
  fixed_map_safety_gate:=true \
  use_l2:=true use_ultrasonic:=true use_cameras:=false \
  perception_texture:=false rviz:=true \
  max_speed:=0.5 max_yaw_rate:=0.4 "$@"
