#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export MONKEYS_FC_UART=/dev/ttyAMA3
export MONKEYS_FC_BAUD=460800
export MONKEYS_LUNA=/dev/ttyAMA2

export MONKEYS_RAW_UNIFIED_PUBLISH="${MONKEYS_RAW_UNIFIED_PUBLISH:-1}"

echo "============================================================"
echo " JT-ZERO — FLIGHT RUNTIME + WEB"
echo "============================================================"
echo "Сеть этим файлом НЕ изменяется."
echo "Web UI: http://192.168.2.1:8080 (в полётной сети)"
echo "Mission Planner: UDPCl -> 192.168.2.1:14550"
echo "Ctrl+C — остановить runtime/web/router."
echo "============================================================"

exec bash "$ROOT/scripts/run_web.sh"
