#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec ./pack_navigation_server.sh "$@"
