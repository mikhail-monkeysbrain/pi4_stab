#!/usr/bin/env python3
"""Offline spatial LK affine consistency audit. No FC/runtime modifications."""
import argparse
import csv
import math
from pathlib import Path
import cv2
import numpy as np

def fit(p,q):
    if len(p)<15:return None
    A,mask=cv2.estimateAffinePartial2D(p,q,method=cv2.RANSAC,
                                      ransacReprojThreshold=2.0,
                                      maxIters=1000,confidence=0.99)
    if A is None or mask is None or int(mask.sum())<12:return None
    scale=math.hypot(float(A[0,0]),float(A[1,0]))
    if not math.isfinite(scale) or scale<=0:return None
    pred=p@A[:,:2].T+A[:,2]
    good=mask.ravel().astype(bool)
    rms=float(np.sqrt(np.mean(np.sum((pred[good]-q[good])**2,axis=1))))
    return math.log(scale),rms,int(good.sum())

def process(rows,writer,acc,grid):
    if not rows:return
    r=rows[0]
    p=np.array([[float(x["x0_px"]),float(x["y0_px"])] for x in rows],np.float64)
    q=np.array([[float(x["x1_px"]),float(x["y1_px"])] for x in rows],np.float64)
    valid=np.isfinite(p).all(axis=1)&np.isfinite(q).all(axis=1)
    p,q=p[valid],q[valid]
    if len(p)<30:return
    # Use observed spatial extents from camera principal point (640x480 acquisition).
    cx,cy=float(r["cx"]),float(r["cy"])
    width,height=2*cx,2*cy
    if width<=0 or height<=0:return
    global_fit=fit(p,q)
    if global_fit is None:return
    values=[]
    for gy in range(grid):
        for gx in range(grid):
            sel=(p[:,0]>=gx*width/grid)&(p[:,0]<(gx+1)*width/grid)&(p[:,1]>=gy*height/grid)&(p[:,1]<(gy+1)*height/grid)
            f=fit(p[sel],q[sel])
            if f is None:continue
            values.append((gx,gy,int(sel.sum()),*f))
    if len(values)<max(2,grid):return
    logs=np.array([x[3] for x in values])
    spread=float(np.std(logs))
    g_log,g_rms,g_in=global_fit
    writer.writerow(dict(frame=int(r["frame"]),t1_ns=r["t1_ns"],points=len(p),
                         cells=len(values),global_logscale=g_log,
                         global_rms_px=g_rms,cell_logscale_std=spread,
                         cell_logscale_min=float(logs.min()),
                         cell_logscale_max=float(logs.max()),
                         cell_logscale_median=float(np.median(logs))))
    acc.append((int(r["t1_ns"]),g_log,spread,len(values)))

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run_dir",type=Path)
    ap.add_argument("--grid",type=int,default=2)
    args=ap.parse_args()
    if not 2<=args.grid<=4:ap.error("grid must be 2..4")
    src=args.run_dir/"visual_z_lk_pairs.csv"
    dst=args.run_dir/"visual_z_spatial_consistency.csv"
    cols=["frame","t1_ns","points","cells","global_logscale",
          "global_rms_px","cell_logscale_std","cell_logscale_min",
          "cell_logscale_max","cell_logscale_median"]
    acc=[]
    with src.open(newline="") as f,dst.open("w",newline="") as o:
        reader=csv.DictReader(f);writer=csv.DictWriter(o,fieldnames=cols)
        writer.writeheader()
        current=None;group=[]
        for row in reader:
            frame=int(row["frame"])
            if current is not None and frame!=current:
                process(group,writer,acc,args.grid);group=[]
            current=frame;group.append(row)
        process(group,writer,acc,args.grid)
    print(f"OUTPUT={dst} VALID_FRAMES={len(acc)}")
    if not acc:return
    spread=np.array([x[2] for x in acc])
    print(f"CELL_LOGSCALE_STD median={np.median(spread):.7f} p95={np.percentile(spread,95):.7f} max={spread.max():.7f}")
    print(f"CELLS median={np.median([x[3] for x in acc]):.1f}")
    t0=acc[0][0]
    for lo,hi in ((0,12),(12,20),(20,28),(28,36),(36,45)):
        seg=[x for x in acc if lo<=(x[0]-t0)*1e-9<hi]
        if seg:
            print(f"WINDOW={lo}-{hi}s frames={len(seg)} median_spread={np.median([x[2] for x in seg]):.7f} median_global_logscale={np.median([x[1] for x in seg]):.7f}")
    print("CAUTION: cells cover different scene regions; local similarity estimates may be noisy.")
    print("Spread alone cannot identify tilt, relief, calibration, or true height.")
    print("No WORKED5/FC changes.")

if __name__=="__main__":main()
