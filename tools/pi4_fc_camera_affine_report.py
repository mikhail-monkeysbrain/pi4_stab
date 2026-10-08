#!/usr/bin/env python3
"""Offline provisional FC->RPi monotonic mapping and camera overlap report.
FC RX latency is unmeasured: absolute alignment is NOT calibrated.
No FC transmission and no WORKED5 changes.
"""
import argparse
import csv
from pathlib import Path
from statistics import median


def affine(points):
    x0,y0=points[0]
    xs=[(x-x0)/1e9 for x,y in points]
    ys=[(y-y0)/1e9 for x,y in points]
    xm=sum(xs)/len(xs);ym=sum(ys)/len(ys)
    den=sum((x-xm)**2 for x in xs)
    slope=sum((x-xm)*(y-ym) for x,y in zip(xs,ys))/den
    offset=ym-slope*xm
    residual=[(y-offset-slope*x)*1000 for x,y in zip(xs,ys)]
    return x0,y0,slope,offset,residual


def pct(v,q):
    s=sorted(v)
    return s[round((len(s)-1)*q)]


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("directory",type=Path)
    args=ap.parse_args()
    fc=[];camera=[]
    with (args.directory/"fc.csv").open(newline="") as f:
        for r in csv.DictReader(f):
            if r.get("msg_type")!="RAW_IMU" or not r.get("fc_time_usec"):
                continue
            fc.append((int(r["fc_time_usec"])*1000,int(r["recv_mono_ns"])))
    with (args.directory/"camera_timestamps.csv").open(newline="") as f:
        for r in csv.DictReader(f):
            if r.get("sensor_ts_ns"):
                camera.append(int(r["sensor_ts_ns"]))
    fc.sort()
    if len(fc)<30 or not camera:
        raise SystemExit("ERROR: insufficient FC RAW_IMU or camera timestamps")
    x0,y0,slope,off,res=affine(fc)
    med=median(res);mad=median(abs(v-med) for v in res)
    threshold=max(5.,6*1.4826*mad)
    clean=[point for point,r in zip(fc,res) if abs(r-med)<=threshold]
    x0,y0,slope,off,res=affine(clean)
    def project(fc_ns):
        return y0+round((off+slope*(fc_ns-x0)/1e9)*1e9)
    start=max(min(camera),project(clean[0][0]))
    end=min(max(camera),project(clean[-1][0]))
    print(f"FC_TO_PI_PROVISIONAL rows={len(fc)} inliers={len(clean)} "
          f"pi_ns_per_fc_ns={slope:.9f} fc_over_pi_ppm={(1/slope-1)*1e6:.1f} "
          f"residual_abs_p95_ms={pct([abs(v) for v in res],.95):.3f}")
    print(f"MODEL_ANCHOR fc_ns={x0} pi_rx_ns={y0} offset_s={off:.9f}")
    print(f"CAMERA_OVERLAP camera_frames={len(camera)} "
          f"camera_first_ns={min(camera)} camera_last_ns={max(camera)} "
          f"projected_fc_first_ns={project(clean[0][0])} "
          f"projected_fc_last_ns={project(clean[-1][0])} "
          f"overlap_s={max(0,end-start)/1e9:.3f}")
    print("WARNING: mapping uses FC UART RECEIVE time; it includes unknown transport latency.")
    print("WARNING: do not use this offset for flight or IMU rotation compensation.")


if __name__=="__main__":
    main()
