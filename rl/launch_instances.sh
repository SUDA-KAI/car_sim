#!/usr/bin/env bash
set -euo pipefail
count="${1:-1}"
if ! [[ "$count" =~ ^[1-9][0-9]*$ ]]; then
  echo "usage: $0 INSTANCE_COUNT [launch arguments...]" >&2
  exit 2
fi
shift || true
root="$(cd "$(dirname "$0")" && pwd)"
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT INT TERM
for ((i=0; i<count; i++)); do
  RL_INSTANCE_ID="$i" "$root/run_rl_sim.sh" "$@" >"/tmp/patrol_rl_${i}.log" 2>&1 &
  pids+=("$!")
done
echo "Started $count instances. Logs: /tmp/patrol_rl_<id>.log"
wait
