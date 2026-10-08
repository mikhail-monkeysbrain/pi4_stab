#!/usr/bin/env python3
"""Read-only Pi4 WORKED5 migration readiness audit. No flight output or FC TX."""
import json
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parent.parent
checks = []

def check(label, ok, detail):
    checks.append((label, bool(ok), detail))
    print(f"{'PASS' if ok else 'BLOCKED'} {label}: {detail}")

check("WORKED5_FROZEN", (root/"src/worked5_estimator.hpp").is_file(),
      "original estimator present; not modified by this audit")
check("OV5647_LIBCAMERA_SOURCE",
      (root/"tools/ov5647_libcamera_worked5_threaded.cpp").is_file(),
      "direct libcamera diagnostic available")
check("FC_READONLY_SOURCE",
      (root/"tools/pi4_fc_readonly_capture.py").is_file(),
      "MAVLink RX-only diagnostic available")
check("FC_UART", Path("/dev/serial0").exists(), "/dev/serial0")
try:
    result = subprocess.run(["pkg-config","--modversion","libcamera","opencv4"],
                            capture_output=True,text=True,timeout=5)
    check("LIBCAMERA_OPENCV",result.returncode==0,
          result.stdout.strip().replace("\n"," / ") or result.stderr.strip())
except (OSError, subprocess.TimeoutExpired) as exc:
    check("LIBCAMERA_OPENCV",False,str(exc))
check("PRODUCTION_CAMERA_ADAPTER",False,
      "scripts/run.sh still expects OV9281 V4L2; direct OV5647 path is diagnostic")
check("REAL_HEIGHT_SOURCE",False,
      "no measured Pi4 altitude source confirmed; synthetic height is not flight-safe")
check("OV5647_CALIBRATION",False,
      "K/D in direct diagnostic are provisional; independent calibration required")
check("FC_FLOW_PUBLISHER",False,
      "Pi4 diagnostics intentionally send no optical flow to FC")
print("READINESS",f"passed={sum(ok for _,ok,_ in checks)}",
      f"blocked={sum(not ok for _,ok,_ in checks)}",
      "flight_ready=NO", "no_fc_tx=1")
