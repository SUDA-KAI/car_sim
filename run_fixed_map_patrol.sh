#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

exec ./run_fixed_map_navigation.sh \
  patrol:=true \
  "route:=$PWD/config/patrol_map_20260817_154310_filtered.yaml" \
  "patrol_report:=$PWD/artifacts/fixed_map_patrol.json" \
  "$@"
