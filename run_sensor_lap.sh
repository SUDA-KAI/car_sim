#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ../install/local_setup.bash
exec /usr/bin/python3 scripts/run_sensor_lap.py "$@"
