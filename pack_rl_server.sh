#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
out="${1:-$root/dist/gz_patrol_rl.tar.gz}"
mkdir -p "$(dirname "$out")"
# The deploy archive deliberately contains only runtime data.  Regeneration
# sources, historical results, drivers and host-specific sensor previews stay in
# the development workspace.
tar -C "$root" -czf "$out" \
  --exclude='__pycache__' --exclude='*.pyc' \
  Dockerfile.rl SERVER_README.md \
  config/bridge.yaml \
  maps/map_20260817_154310_filtered.pgm \
  maps/map_20260817_154310_filtered.yaml \
  models/car_body.urdf models/car_properties.json \
  models/patrol_car.sdf models/patrol_car.urdf models/meshes/car_body_m.stl \
  rl \
  worlds/map_20260817_154310_filtered.json \
  worlds/map_20260817_154310_filtered.obj \
  worlds/map_20260817_154310_filtered.sdf
echo "$out"
