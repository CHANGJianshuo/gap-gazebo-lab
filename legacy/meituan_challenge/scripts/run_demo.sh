#!/usr/bin/env bash
set -e
MTC_SCRIPT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$MTC_SCRIPT_ROOT"
source scripts/env.sh
exec python3 scripts/run_demo.py "$@"
