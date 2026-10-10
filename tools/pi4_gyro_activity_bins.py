#!/usr/bin/env python3
"""Offline detection of gyro activity in existing FC CSV. No new hardware run."""
import argparse
import csv
import math
from pathlib import Path
from statistics import median


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("directory")
    ap.add_argument("--bin",type=float,default=2.0)
    args=ap.parse_args()
    path=Path(args.directory)/"fc.csv"
    with path.open(newline="") as f:
        rows=[r for r in csv.DictReader(f) if r.get("msg_type")=="ATTITUDE"]
    data=[]
    for row in rows:
        try:
            t=int(row["recv_mono_ns"])*1e-9
            x=float(row["rollspeed"])
            y=float(row["pitchspeed"])
            z=float(row["yawspeed"])
            if all(math.isfinite(v) for v in (t,x,y,z)):
                data.append((t,math.hypot(x,y),abs(z)))
        except (KeyError,ValueError):
            pass
    if not data:
        raise SystemExit("NO ATTITUDE DATA")
    t0=data[0][0]
    print("GYRO_ACTIVITY ATTITUDE 2D roll/pitch angular rate (rad/s)")
    n=math.ceil((data[-1][0]-t0)/args.bin)
    for i in range(n):
        subset=[(xy,z) for t,xy,z in data if i*args.bin <= t-t0 < (i+1)*args.bin]
        if not subset:
            continue
        xy=sorted(v[0] for v in subset)
        z=sorted(v[1] for v in subset)
        print(f"BIN {i*args.bin:5.1f}-{(i+1)*args.bin:5.1f}s "
              f"n={len(subset):3d} xy_med={median(xy):.5f} "
              f"xy_p95={xy[int(.95*(len(xy)-1))]:.5f} "
              f"yaw_p95={z[int(.95*(len(z)-1))]:.5f}")
    print("NOTE timestamps are UART receive times; no camera alignment claimed.")


if __name__=="__main__":
    main()
