#!/usr/bin/env python3
"""Offline homography rank-one planar consistency after ATTITUDE derotation.

Uses recorded LK pairs. This is a diagnostic, NOT an altitude estimator.
"""
import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np


def euler_ned(roll, pitch, yaw):
    cr,sr=math.cos(roll),math.sin(roll)
    cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                     [-sp,cp*sr,cp*cr]],dtype=float)


def analyze(rows, mount, min_points):
    r=rows[0]
    if not (int(r["att0_valid"]) and int(r["att1_valid"])):
        return None
    K=np.array([[float(r["fx"]),0,float(r["cx"])],
                [0,float(r["fy"]),float(r["cy"])],[0,0,1]],dtype=float)
    if K[0,0]<=0 or K[1,1]<=0:return None
    p=np.array([[float(v["x0_px"]),float(v["y0_px"])] for v in rows],dtype=float)
    q=np.array([[float(v["x1_px"]),float(v["y1_px"])] for v in rows],dtype=float)
    ok=np.isfinite(p).all(axis=1)&np.isfinite(q).all(axis=1)
    p,q=p[ok],q[ok]
    if len(p)<min_points:return None
    # K is provisional OV9281-based; distortion unavailable for OV5647.
    pn=cv2.undistortPoints(p.reshape(-1,1,2),K,None).reshape(-1,2)
    qn=cv2.undistortPoints(q.reshape(-1,1,2),K,None).reshape(-1,2)
    H,mask=cv2.findHomography(pn,qn,cv2.RANSAC,0.003,maxIters=2000,confidence=0.995)
    if H is None or mask is None or int(mask.sum())<min_points:return None
    a0=euler_ned(*(float(r[k]) for k in ("roll0","pitch0","yaw0")))
    a1=euler_ned(*(float(r[k]) for k in ("roll1","pitch1","yaw1")))
    R=mount.T@a1.T@a0@mount
    # Homography is projective: optimize its scalar multiplier before testing H-R.
    # For rank-one residual, two smallest singular values should approach zero.
    candidates=np.linspace(0.97,1.03,241)
    best=None
    for k in candidates:
        D=k*H-R
        s=np.linalg.svd(D,compute_uv=False)
        cost=math.hypot(s[1],s[2])
        if best is None or cost<best[0]:best=(cost,k,s,D)
    _,scale,s,D=best
    # A tiny residual makes normal direction ill-conditioned.
    ratio=float(math.hypot(s[1],s[2])/max(s[0],1e-12))
    signal=float(s[0])
    _,_,vt=np.linalg.svd(D)
    n=vt[0]
    # Sign ambiguous; only unsigned tilt from camera optical axis is reported.
    tilt=math.degrees(math.acos(float(np.clip(abs(n[2]),0,1))))
    return dict(frame=r["frame"],t1_ns=r["t1_ns"],pairs=len(p),
                inliers=int(mask.sum()),projective_scale=scale,
                singular_1=s[0],singular_2=s[1],singular_3=s[2],
                rank1_ratio=ratio,rank1_signal=signal,
                normal_tilt_deg=tilt)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run_dir",type=Path)
    ap.add_argument("--min-points",type=int,default=40)
    args=ap.parse_args()
    # Camera OpenCV x=body right, y=body backward, z=body down (FRD).
    mount=np.array([[0,-1,0],[1,0,0],[0,0,1]],dtype=float)
    src=args.run_dir/"visual_z_lk_pairs.csv"
    dst=args.run_dir/"visual_z_planar_rank1.csv"
    fields=["frame","t1_ns","pairs","inliers","projective_scale",
            "singular_1","singular_2","singular_3","rank1_ratio",
            "rank1_signal","normal_tilt_deg"]
    data=[]
    with src.open(newline="") as f,dst.open("w",newline="") as o:
        reader=csv.DictReader(f);writer=csv.DictWriter(o,fieldnames=fields)
        writer.writeheader()
        group=[];last=None
        def flush():
            if not group:return
            result=analyze(group,mount,args.min_points)
            if result is not None:
                writer.writerow(result);data.append(result)
        for row in reader:
            frame=row["frame"]
            if last is not None and frame!=last:
                flush();group=[]
            group.append(row);last=frame
        flush()
    print(f"OUTPUT={dst} VALID_FRAMES={len(data)}")
    if not data:return
    t0=int(data[0]["t1_ns"])
    for lo,hi in ((0,12),(12,20),(20,28),(28,36),(36,45)):
        seg=[v for v in data if lo<=(int(v["t1_ns"])-t0)*1e-9<hi]
        if not seg:continue
        def med(k):return float(np.median([v[k] for v in seg]))
        print(f"WINDOW={lo}-{hi}s frames={len(seg)} "
              f"rank1_ratio_med={med('rank1_ratio'):.5f} "
              f"signal_med={med('rank1_signal'):.6f} "
              f"normal_tilt_med_deg={med('normal_tilt_deg'):.2f} "
              f"inliers_med={med('inliers'):.0f}")
    print("WARNING: per-frame projective scale search limited to 0.97..1.03.")
    print("Near-static normal tilt is unobservable; do not interpret as physical tilt.")
    print("OV5647 intrinsics, distortion, mounting, and ATTITUDE synchronization uncalibrated.")
    print("Rank-one consistency is necessary, not sufficient, for a single physical plane.")
    print("No changes to runtime, WORKED5 or FC.")


if __name__=="__main__":
    main()
