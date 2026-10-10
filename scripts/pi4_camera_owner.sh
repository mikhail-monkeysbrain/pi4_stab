#!/usr/bin/env bash
set -euo pipefail
echo "=== Pi4 camera owners ==="
for dev in /dev/media0 /dev/media2 /dev/video0; do
  if [[ -e "$dev" ]]; then
    echo "--- $dev ---"
    fuser -v "$dev" 2>&1 || true
  fi
done
echo "=== Camera and runtime processes ==="
ps -eo pid,ppid,stat,cmd | grep -E '[p]i4_(web_runtime|original_web|ov5647_adapter)|[r]picam-|[l]ibcamera-|[c]amera-streamer|[m]otion' || true
echo "=== Listening web ports ==="
ss -ltnp '( sport = :8080 or sport = :18080 )' || true
