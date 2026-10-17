#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-/tmp/monkeysstab_csi_gray_probe}"
g++ -std=c++17 -O2 -pthread -I"$ROOT/src" \
  "$ROOT/tools/csi_gray_probe.cpp" -o "$OUT" \
  $(pkg-config --cflags --libs libcamera opencv4)
echo "BUILD PASS: $OUT"
"$OUT"
