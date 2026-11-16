#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
instance="${RL_INSTANCE_ID:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((42 + instance))}"
export IGN_PARTITION="${IGN_PARTITION:-patrol_rl_${instance}}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/patrol_rl_logs_${instance}}"
export IGN_LOG_PATH="${IGN_LOG_PATH:-/tmp/patrol_rl_ign_${instance}}"
mkdir -p "$ROS_LOG_DIR"
mkdir -p "$IGN_LOG_PATH"
set +u
source /opt/ros/humble/setup.bash
set -u
echo "RL instance=$instance ROS_DOMAIN_ID=$ROS_DOMAIN_ID IGN_PARTITION=$IGN_PARTITION"
exec ros2 launch "$root/rl/launch/rl_sim.launch.py" "$@"
