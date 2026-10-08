#!/usr/bin/env bash
# Build and run the real frozen WORKED5 estimator on CSI OV5647 frames.
# Diagnostic only: synthetic height, provisional intrinsics, no FC output.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
OUT=/tmp/pi4_ov5647_worked5_diag
g++ -std=c++17 -O2 -DNDEBUG -pthread -I"$ROOT/src" \
  "$ROOT/tools/ov5647_worked5_diagnostic.cpp" \
  -o "$OUT" $(pkg-config --cflags --libs opencv4)
echo "=== CPU and temperature ==="
vcgencmd measure_temp || true
vcgencmd get_throttled || true
echo "=== WORKED5 (15s) ==="
timeout 25s "$OUT"
echo "=== After test ==="
vcgencmd measure_temp || true
vcgencmd get_throttled || true
