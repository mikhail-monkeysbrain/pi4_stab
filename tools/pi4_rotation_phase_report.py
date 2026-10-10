#!/usr/bin/env python3
"""Offline 3-phase motion audit for OV5647 WORKED5 and FC IMU.
Separate timelines; no claim of camera/FC synchronization or metric displacement.
"""
import argparse
import csv
import math
from pathlib import Path
from statistics import median


def load(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def number(row, name):
    try:
        x=float(row[name])
        return x if math.isfinite(x) else None
    except (KeyError,ValueError,TypeError):
        return None


def report(label, records, time_key, fields, duration):
    timed=[]
    for row in records:
        t=number(row,time_key)
        if t is not None:
            timed.append((t,row))
    if not timed:
        print(f"{label} NO_TIMESTAMP")
        return
    t0=min(t for t,_ in timed)
    print(f"{label} rows={len(timed)} time_span_s={(max(t for t,_ in timed)-t0)*1e-9:.2f}")
    for phase,lo,hi in (("START",0,10),("MOTION",10,20),("END",20,30)):
        subset=[row for t,row in timed if lo<=((t-t0)*1e-9)<hi]
        stats=[]
        for field in fields:
            values=[number(row,field) for row in subset]
            values=[abs(v) for v in values if v is not None]
            if values:
                stats.append(f"{field}_abs_med={median(values):.6g}")
                stats.append(f"{field}_abs_p95={sorted(values)[int((len(values)-1)*.95)]:.6g}")
        print(f"{label} {phase} rows={len(subset)} "+ " ".join(stats))
    print(f"{label} NOTE phases use this stream's own first timestamp; "
          "camera and FC starts are NOT aligned")


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("directory")
    args=ap.parse_args()
    root=Path(args.directory)
    steps=load(root/"worked5_steps.csv")
    fc=load(root/"fc.csv")
    report("WORKED5",steps,"sensor_ts_ns",
           ("du_norm","dv_norm","yaw_per_s","scale_per_s"),30)
    # FC receive monotonic timestamp is on the Pi clock, but reflects UART
    # arrival time, not the IMU sample time. Use it only for coarse phases.
    for kind in ("ATTITUDE","RAW_IMU","SCALED_IMU"):
        rows=[row for row in fc if row.get("msg_type")==kind]
        fields=(("rollspeed","pitchspeed","yawspeed") if kind=="ATTITUDE"
                else ("xgyro","ygyro","zgyro"))
        report(kind,rows,"recv_mono_ns",fields,30)
    print("LIMITATION visual fields are frozen WORKED5 estimator outputs; "
          "they are not independent camera angular rates.")
    print("LIMITATION FC RAW_IMU/SCALED_IMU gyro units differ from ATTITUDE rates; "
          "do not compare their numeric magnitudes directly.")
    print("LIMITATION phase alignment is approximate; no AGL and no flight output.")


if __name__=="__main__":
    main()
