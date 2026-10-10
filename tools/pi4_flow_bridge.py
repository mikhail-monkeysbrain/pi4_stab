#!/usr/bin/env python3
"""Offline WORKED5-to-MAVLink publisher boundary, intentionally no serial TX.

This bridge validates the producer CSV schema and writes transport-ready
normalized flow records. It never claims metric displacement or sends MAVLink.
"""
import argparse
import csv
import math
from pathlib import Path

FIELDS=("sensor_ts_ns","dt_s","points","du_norm","dv_norm","scale_per_s","yaw_per_s","metric_valid")


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("steps",type=Path)
    parser.add_argument("--out",type=Path,default=None)
    args=parser.parse_args()
    target=args.out or args.steps.with_name("flow_bridge.csv")
    if target.resolve()==args.steps.resolve():
        parser.error("output must differ from input")
    valid=0
    rejected=0
    with args.steps.open(newline="") as source, target.open("w",newline="") as output:
        reader=csv.DictReader(source)
        if not set(FIELDS).issubset(reader.fieldnames or []):
            parser.error("WORKED5 CSV schema mismatch")
        writer=csv.DictWriter(output,fieldnames=["sensor_ts_ns","dt_s","points",
                                                 "du_norm","dv_norm","scale_per_s",
                                                 "yaw_per_s","metric_valid","publish_allowed"])
        writer.writeheader()
        previous=0
        for row in reader:
            try:
                ts=int(row["sensor_ts_ns"])
                dt=float(row["dt_s"])
                values=[float(row[k]) for k in ("du_norm","dv_norm","scale_per_s","yaw_per_s")]
                points=int(row["points"])
                if ts<=previous or not 0<dt<0.2 or points<20 or not all(map(math.isfinite,values)):
                    raise ValueError("invalid optical flow record")
                previous=ts
            except (ValueError,TypeError,OverflowError):
                rejected+=1
                continue
            writer.writerow({k:row[k] for k in FIELDS}|{"publish_allowed":0})
            valid+=1
    print(f"FLOW_BRIDGE records={valid} rejected={rejected} output={target}")
    print("MAVLINK_TX=OFF METRIC_HEIGHT=UNAVAILABLE FLOW_PUBLISH=BLOCKED")
    return 0 if valid>0 and rejected==0 else 1


if __name__=="__main__":
    raise SystemExit(main())
