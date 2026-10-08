#!/usr/bin/env python3
"""Robust FC boot time vs Pi receive time diagnostic; offline RX-only CSV."""
import argparse
import csv
from pathlib import Path
from statistics import median


def fit(points):
    x0,y0=points[0]
    xs=[(x-x0)*1e-9 for x,y in points]
    ys=[(y-y0)*1e-9 for x,y in points]
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys)
    den=sum((x-mx)**2 for x in xs)
    slope=sum((x-mx)*(y-my) for x,y in zip(xs,ys))/den
    offset=my-slope*mx
    residual=[(y-offset-slope*x)*1000 for x,y in zip(xs,ys)]
    return slope,residual


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("csv",type=Path)
    args=ap.parse_args()
    groups={"ATTITUDE":[],"SCALED_IMU":[]}
    with args.csv.open(newline="") as f:
        for row in csv.DictReader(f):
            kind=row.get("msg_type")
            if kind not in groups or not row.get("fc_time_boot_ms"):
                continue
            try:
                groups[kind].append((int(row["recv_mono_ns"]),int(float(row["fc_time_boot_ms"])*1e6)))
            except ValueError:
                pass
    for kind,rows in groups.items():
        if len(rows)<30:
            print(f"ROBUST_CLOCK type={kind} insufficient_rows={len(rows)}")
            continue
        slope,res=fit(rows)
        center=median(res)
        mad=median(abs(v-center) for v in res)
        threshold=max(5.,6*1.4826*mad)
        kept=[r for r,v in zip(rows,res) if abs(v-center)<=threshold]
        if len(kept)<20:
            print(f"ROBUST_CLOCK type={kind} insufficient_inliers")
            continue
        robust,robust_res=fit(kept)
        largest=sorted(enumerate(res),key=lambda v:abs(v[1]),reverse=True)[:5]
        print(f"ROBUST_CLOCK type={kind} rows={len(rows)} inliers={len(kept)} "
              f"outliers={len(rows)-len(kept)} threshold_ms={threshold:.3f} "
              f"ols_ppm={(slope-1)*1e6:.1f} filtered_ppm={(robust-1)*1e6:.1f} "
              f"inlier_abs_p95_ms={sorted(abs(x) for x in robust_res)[round(.95*(len(robust_res)-1))]:.3f}")
        for i,v in largest:
            print(f"  OUTLIER type={kind} index={i} residual_ms={v:.3f} "
                  f"recv_mono_ns={rows[i][0]} fc_boot_ns={rows[i][1]}")
    print("NOTE: filtered rate remains receive-time based, not verified hardware clock drift.")


if __name__=="__main__":
    main()
