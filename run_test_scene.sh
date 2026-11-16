#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
scene="${1:-narrow_turns}"
mode="${2:-auto}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ $# -gt 0 ]]; then shift; fi
# Common scene names now select the 1.7 m suite; wide_* explicitly selects history.
case "$scene" in
  turns|ramp_6deg|ramp_12deg|slip_normal|slip_low|slip_ice) scene="narrow_$scene";;
  wide_turns|wide_ramp_6deg|wide_ramp_12deg|wide_slip_normal|wide_slip_low|wide_slip_ice) scene="${scene#wide_}";;
esac
case "$scene" in turns|ramp_6deg|ramp_12deg|slip_normal|slip_low|slip_ice|narrow_17|narrow_17_baseline|narrow_turns|narrow_ramp_6deg|narrow_ramp_12deg|narrow_slip_normal|narrow_slip_low|narrow_slip_ice) ;;
  *) echo '场景：narrow_turns / narrow_ramp_6deg / narrow_ramp_12deg / narrow_slip_normal / narrow_slip_low / narrow_slip_ice / narrow_17 / narrow_17_baseline'; exit 2;;
esac
case "$mode" in
  auto) control=(teleop:=false nav:=true patrol:=true);;
  manual) control=(teleop:=true nav:=false patrol:=false);;
  world) control=(teleop:=false nav:=false patrol:=false);;
  *) echo '模式：auto（自主巡逻）/ manual（手柄）/ world（仅环境）'; exit 2;;
esac
exec ./run_sim.sh "$@" "world:=$PWD/test_scenes/worlds/$scene.sdf" "${control[@]}" record_metrics:=true
