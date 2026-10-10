#!/usr/bin/env python3
"""Pi4 integrated runtime bench launcher. No flight output or MAVLink flow publishing."""
import argparse
import csv
from pathlib import Path
import subprocess
import sys
import time


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--seconds",type=int,default=8)
    ap.add_argument("--port",default="/dev/serial0")
    ap.add_argument("--baud",type=int,default=460800)
    args=ap.parse_args()
    if not 5 <= args.seconds <= 120:
        ap.error("--seconds must be 5..120")
    root=Path(__file__).resolve().parent.parent
    out=Path("/tmp")/f"pi4_runtime_{time.strftime('%Y%m%d_%H%M%S')}"
    cmd=[sys.executable,str(root/"tools/pi4_adapter_imu_joint_progress.py"),
         "--seconds",str(args.seconds),"--port",args.port,
         "--baud",str(args.baud),"--out",str(out)]
    print("PI4_RUNTIME mode=BENCH publisher=OFF flight_output=OFF",flush=True)
    rc=subprocess.run(cmd,cwd=root,check=False).returncode
    steps=out/"worked5_steps.csv"
    fc=out/"fc.csv"
    counts={}
    if fc.exists():
        with fc.open(newline="") as handle:
            for row in csv.DictReader(handle):
                kind=row["msg_type"]
                counts[kind]=counts.get(kind,0)+1
    visual=0
    if steps.exists():
        with steps.open(newline="") as handle:
            visual=sum(1 for _ in csv.DictReader(handle))
    print("PI4_RUNTIME_RESULT",f"rc={rc}",f"worked5={visual}",
          f"attitude={counts.get('ATTITUDE',0)}",
          f"raw_imu={counts.get('RAW_IMU',0)}",
          f"scaled_imu={counts.get('SCALED_IMU',0)}",
          "flow_publisher=OFF","flight_ready=NO",f"dir={out}",flush=True)
    if rc or visual==0 or not all(counts.get(k,0)>0 for k in ("ATTITUDE","RAW_IMU","SCALED_IMU")):
        print("BLOCKER runtime bench failed",flush=True)
        return 1
    print("BENCH_PASS; BLOCKERS: metric AGL, calibration, flow publisher, EKF3 verification",flush=True)
    return 0


if __name__=="__main__":
    raise SystemExit(main())
