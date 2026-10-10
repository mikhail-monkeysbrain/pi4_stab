#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PORT="${MONKEYS_WEB_PORT:-8080}"
PY="${MONKEYS_PYTHON:-$HOME/mavlink-test-venv/bin/python}"
if [[ ! -x "$PY" ]]; then PY=python3; fi
exec "$PY" -u "$ROOT/tools/pi4_original_web.py" \
  --serial "${MONKEYS_FC_UART:-/dev/serial0}" \
  --baud "${MONKEYS_FC_BAUD:-460800}" \
  --port "$PORT"
