#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
export IGN_PARTITION="${SIM_IGN_PARTITION:-patrol_sensor_preview}"
/usr/bin/python3 scripts/build_sensor_preview.py
exec ign gazebo -r -v 3 --gui-config "$PWD/config/sensor_preview_gui.config" "$PWD/worlds/sensor_mounting_preview.sdf" "$@"
