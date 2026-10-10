#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PORT="${MONKEYS_WEB_PORT:-8080}"
PY="${MONKEYS_PYTHON:-$HOME/mavlink-test-venv/bin/python}"
if [[ ! -x "$PY" ]]; then PY=python3; fi
export MONKEYS_PI4_RUNTIME_SAFE=1
export MONKEYS_PI4_BARO="${MONKEYS_PI4_BARO:-1}"
export MONKEYS_PI4_BARO_H0="${MONKEYS_PI4_BARO_H0:-0.18}"
export MONKEYS_PI4_OV5647=1
export MONKEYS_NO_LUNA=1
export MONKEYS_LOCAL_GUI=0
exec "$PY" -u "$ROOT/tools/web_service.py" --host 0.0.0.0 --port "$PORT"
