#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
if [[ $# -eq 0 ]] && /usr/bin/python3 - <<'PY'
import json,sys
from urllib.request import urlopen
try:
    data=json.load(urlopen('http://127.0.0.1:8767/api/state',timeout=1))
    sys.exit(0 if 'main_bracket' in data.get('assets',{}) else 1)
except Exception:
    sys.exit(1)
PY
then
    exec firefox --new-tab http://127.0.0.1:8767
fi
exec /usr/bin/python3 assembly/server.py --open "$@"
