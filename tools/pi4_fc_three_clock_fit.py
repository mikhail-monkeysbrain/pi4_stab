#!/usr/bin/env python3
"""Robust offline FC clock slope using RAW_IMU time_usec and ATTITUDE boot ms.
Receive timestamps are affected by UART latency; no sensor sync claim.
"""
import argparse
import csv
from pathlib import Path
from statistics import median


def fit(points):
    x0,y0=points[0]
    xs=[(x-x0)*1e-9 for x,y in points]
    ys=[(y-y0)*1e-9 for x,y in points]
    xm=sum(xs)/len(xs); ym=sum(ys)/len(ys)
    den=sum((x-xm)**2 for x in xs)
    if den==0: raise ValueError("zero span")
    slope=sum((x-xm)*(y-ym) for x,y in zip(xs,ys))/den
    intercept=ym-slope*xm
    return slope,[(y-intercept-slope*x)*1000 for x,y in zip(xs,ys)]


def report(name,points):
    if len(points)<30:
        print(f"CLOCK_COMPARE type={name} insufficient={len(points)}")
        return
    slope,res=fit(points)
    center=median(res)
    mad=median(abs(x-center) for x in res)
    threshold=max(5.0,6*1.4826*mad)
    clean=[p for p,r in zip(points,res) if abs(r-center)<=threshold]
    slope2,res2=fit(clean)
    p95=sorted(abs(x) for x in res2)[round(.95*(len(res2)-1))]
    print(f"CLOCK_COMPARE type={name} rows={len(points)} inliers={len(clean)} "
          f"ols_ppm={(slope-1)*1e6:.1f} filtered_ppm={(slope2-1)*1e6:.1f} "
          f"residual_abs_p95_ms={p95:.3f} "
          f"first_fc_ns={points[0][1]} last_fc_ns={points[-1][1]}")
    for label,subset in (("first_half",clean[:len(clean)//2]),
                         ("second_half",clean[len(clean)//2:])):
        sub_slope,_=fit(subset)
        print(f"  window={label} filtered_ppm={(sub_slope-1)*1e6:.1f}")


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("directory",type=Path)
    args=ap.parse_args()
    groups={"RAW_IMU":[],"ATTITUDE":[],"SCALED_IMU":[]}
    with (args.directory/"fc.csv").open(newline="") as f:
        for row in csv.DictReader(f):
            kind=row.get("msg_type")
            if kind not in groups: continue
            try:
                rx=int(row["recv_mono_ns"])
                if kind=="RAW_IMU":
                    if not row.get("fc_time_usec"): continue
                    stamp=int(row["fc_time_usec"])*1000
                else:
                    if not row.get("fc_time_boot_ms"): continue
                    stamp=int(row["fc_time_boot_ms"])*1000000
                groups[kind].append((rx,stamp))
            except (ValueError,KeyError):
                continue
    for name,points in groups.items():
        report(name,points)
    print("LIMITATION: RX-time fit includes transport/scheduling latency; no exposure-to-IMU offset measured.")


if __name__=="__main__":
    main()
