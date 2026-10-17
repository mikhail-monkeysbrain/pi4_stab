#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-/tmp/monkeysstab_csi_capture_probe}"
g++ -std=c++17 -O2 -pthread \
  "$ROOT/tools/csi_capture_probe.cpp" -o "$OUT" \
  $(pkg-config --cflags --libs libcamera)
echo "BUILD PASS: $OUT"
echo "Run: $OUT 10 60"
