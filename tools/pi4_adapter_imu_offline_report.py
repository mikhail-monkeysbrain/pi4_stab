#!/usr/bin/env python3
"""Offline clock-rate and data-quality audit. No camera/FC access or MAVLink TX."""
import argparse
import csv
import math
from pathlib import Path
from statistics import median


def read(path):
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def quantile(values, q):
    values=sorted(values)
    if not values:
        return float("nan")
    return values[min(len(values)-1,int((len(values)-1)*q))]


def affine(rows, kind, field, factor):
    pairs=[]
    for row in rows:
        if row["msg_type"]!=kind:
            continue
        try:
            x=float(row[field])*factor
            y=float(row["recv_mono_ns"])*1e-9
        except (ValueError,KeyError):
            continue
        if math.isfinite(x) and math.isfinite(y) and x>0:
            pairs.append((x,y))
    if len(pairs)<20:
        print(f"CLOCK {kind} insufficient={len(pairs)}")
        return
    pairs.sort()
    monotonic=sum(b[0]<=a[0] for a,b in zip(pairs,pairs[1:]))
    # Center the regression to avoid catastrophic cancellation.
    x0=median(x for x,_ in pairs)
    y0=median(y for _,y in pairs)
    def fit(items):
        xx=[x-x0 for x,_ in items]
        yy=[y-y0 for _,y in items]
        mx=sum(xx)/len(xx)
        my=sum(yy)/len(yy)
        denom=sum((x-mx)**2 for x in xx)
        if denom<=0:
            raise ValueError("constant FC timestamps")
        slope=sum((x-mx)*(y-my) for x,y in zip(xx,yy))/denom
        offset=y0+my-slope*(x0+mx)
        return slope,offset
    slope,offset=fit(pairs)
    residual=[y-(slope*x+offset) for x,y in pairs]
    med=median(residual)
    mad=median(abs(r-med) for r in residual)
    limit=max(0.005,6*1.4826*mad)
    clean=[pair for pair,r in zip(pairs,residual) if abs(r-med)<=limit]
    if len(clean)>=20:
        slope,offset=fit(clean)
    residual=[abs(y-(slope*x+offset))*1000 for x,y in clean]
    gaps=[(b[0]-a[0])*1000 for a,b in zip(pairs,pairs[1:])]
    print(f"CLOCK {kind} rows={len(pairs)} inliers={len(clean)} "
          f"fc_over_pi_ppm={(1/slope-1)*1e6:.1f} "
          f"residual_abs_p95_ms={quantile(residual,.95):.3f} "
          f"fc_nonmonotonic={monotonic} "
          f"fc_gap_p95_ms={quantile(gaps,.95):.3f} "
          "sync=UNVERIFIED_UART_RX_LATENCY")


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("directory")
    args=ap.parse_args()
    root=Path(args.directory)
    steps=read(root/"worked5_steps.csv")
    fc=read(root/"fc.csv")
    timestamps=[]
    invalid_dt=0
    for row in steps:
        try:
            ts=int(row["sensor_ts_ns"])
            dt=float(row["dt_s"])
            timestamps.append(ts)
            if not (0<dt<0.2):
                invalid_dt+=1
        except (KeyError,ValueError):
            invalid_dt+=1
    nonmono=sum(b<=a for a,b in zip(timestamps,timestamps[1:]))
    print(f"WORKED5 rows={len(steps)} nonmonotonic={nonmono} invalid_dt={invalid_dt} "
          f"metric_valid={sum(row.get('metric_valid')=='1' for row in steps)}")
    for kind,field,factor in (("ATTITUDE","fc_time_boot_ms",1e-3),
                              ("RAW_IMU","fc_time_usec",1e-6),
                              ("SCALED_IMU","fc_time_boot_ms",1e-3)):
        affine(fc,kind,field,factor)
    print("NOTE FC rate is provisional from UART receive timestamps; no exposure-to-IMU offset inferred.")


if __name__=="__main__":
    main()
