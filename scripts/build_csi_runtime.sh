#!/usr/bin/env bash
# Build-only CSI variant; never replaces the production binary or starts FC.
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAVLINK_ROOT="${MAVLINK_ROOT:-}"
if [[ -z "$MAVLINK_ROOT" ]]; then
  for d in "$ROOT/third_party/mavlink" /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [[ -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_ROOT="$d"; break; fi
  done
fi
[[ -f "$MAVLINK_ROOT/ardupilotmega/mavlink.h" ]] || {
  echo "ОШИБКА: MAVLink headers не найдены" >&2
  exit 2
}
OUT="${1:-/tmp/monkeysstab_optical_flow_csi_buildcheck}"
g++ -std=c++17 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member \
  -DJTZERO_CSI_OV5647 \
  $(pkg-config --cflags opencv4 libcamera) \
  -I"$MAVLINK_ROOT" -I"$ROOT/src" \
  "$ROOT/src/optical_flow_mavlink.cpp" -o "$OUT" \
  $(pkg-config --libs opencv4 libcamera) -lpthread
echo "CSI BUILD PASS: $OUT"
echo "Сборка завершена. Бинарник не запускается автоматически."
