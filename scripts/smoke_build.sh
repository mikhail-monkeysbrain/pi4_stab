#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  for d in "$ROOT/third_party/mavlink" /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then
      MAVLINK_ROOT="$d"
      break
    fi
  done
fi
if [[ -z "${MAVLINK_ROOT:-}" ]]; then
  bash "$ROOT/scripts/bootstrap_dependencies.sh"
  MAVLINK_ROOT="$ROOT/third_party/mavlink"
fi

[[ -f "$MAVLINK_ROOT/ardupilotmega/mavlink.h" ]] || {
  echo "ОШИБКА: MAVLink headers не найдены: $MAVLINK_ROOT" >&2
  exit 2
}

OUT="${1:-/tmp/monkeysstab_optical_flow_buildcheck}"
g++ -std=c++17 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member \
  $(pkg-config --cflags opencv4 libcamera) -I"$MAVLINK_ROOT" -I"$ROOT/src" \
  "$ROOT/src/optical_flow_mavlink.cpp" -o "$OUT" \
  $(pkg-config --libs opencv4 libcamera) -lpthread

echo "BUILD PASS: $OUT"
