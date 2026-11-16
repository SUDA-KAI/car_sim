#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
scene="${1:-narrow_turns}"
if [[ $# -gt 0 ]]; then shift; fi
world="$PWD/test_scenes/worlds/$scene.sdf"
if [[ ! -f "$world" ]]; then
  world="$PWD/worlds/$scene.sdf"
  if [[ ! -f "$world" ]]; then
    echo "Unknown scene: $scene" >&2
    exit 2
  fi
fi
# Dedicated domain and partition keep this server instance away from real ROS.
export SIM_ROS_DOMAIN_ID="${SIM_ROS_DOMAIN_ID:-73}"
export SIM_IGN_PARTITION="${SIM_IGN_PARTITION:-patrol_nav_server}"
exec ./run_sim.sh "world:=$world" gui:=false nav:=true patrol:=false \
  localization:=amcl controller:=mppi use_l2:=true use_d6:=true \
  use_ultrasonic:=true use_assembly:=false sensor_visuals:=false \
  sim_drive_adapter:=true max_yaw_rate:=0.45 sensor_monitor:=false record_metrics:=false "$@"
