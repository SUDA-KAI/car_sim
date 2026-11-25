#!/usr/bin/env bash
# Web3D 仿真导航台 启动脚本
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
export PATH=/usr/bin:$PATH
export GAZEBO_MODEL_DATABASE_URI=""
exec /usr/bin/python3 server.py "$@"
