#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
exec ./run_sim.sh "$@" teleop:=true nav:=false
