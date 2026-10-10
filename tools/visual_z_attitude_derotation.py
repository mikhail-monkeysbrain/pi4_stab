#!/usr/bin/env python3
"""Offline ATTITUDE derotation of LK pairs; diagnostic, not metric altitude."""
import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np


def attitude_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr]
    ])


def fit_scale(x, y):
    H, mask = cv2.findHomography(x, y, cv2.RANSAC, 2.0)
    if H is None or mask is None or mask.sum() < 20:
        return None
    H /= H[2, 2]
    jac = H[:2, :2] - np.outer(H[:2, 2], H[2, :2])
    det = float(np.linalg.det(jac))
    if det <= 0 or not math.isfinite(det):
        return None
    pred = cv2.perspectiveTransform(x.reshape(-1, 1, 2), H).reshape(-1, 2)
    sel = mask.ravel().astype(bool)
    rms = float(np.sqrt(np.mean(np.sum((pred[sel]-y[sel])**2, axis=1))))
    return math.log(math.sqrt(det)), rms, int(sel.sum())


def evaluate(rows, mount):
    r = rows[0]
    if int(r["att0_valid"]) != 1 or int(r["att1_valid"]) != 1:
        return None
    fx, fy, cx, cy = (float(r[k]) for k in ("fx", "fy", "cx", "cy"))
    if fx <= 0 or fy <= 0:
        return None
    K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=float)
    p = np.array([[float(v["x0_px"]),float(v["y0_px"])] for v in rows])
    q = np.array([[float(v["x1_px"]),float(v["y1_px"])] for v in rows])
    good = np.isfinite(p).all(axis=1)&np.isfinite(q).all(axis=1)
    p,q=p[good],q[good]
    if len(p)<20:
        return None
    p = cv2.undistortPoints(p.reshape(-1,1,2), K, None).reshape(-1,2)
    q = cv2.undistortPoints(q.reshape(-1,1,2), K, None).reshape(-1,2)
    a0 = attitude_matrix(*(float(r[k]) for k in ("roll0","pitch0","yaw0")))
    a1 = attitude_matrix(*(float(r[k]) for k in ("roll1","pitch1","yaw1")))
    # Body FRD -> NED attitude; mount maps camera OpenCV -> body FRD.
    rot = mount.T @ a1.T @ a0 @ mount
    rays = np.column_stack((p, np.ones(len(p))))
    rotated = (rot @ rays.T).T
    good = np.isfinite(rotated).all(axis=1)&(rotated[:,2]>0.1)
    p_rot = rotated[good,:2]/rotated[good,2,None]
    q = q[good]
    if len(q)<20:
        return None
    fitted=fit_scale(p_rot,q)
    if fitted is None:
        return None
    logscale,rms,inliers=fitted
    return logscale,rms,inliers,len(q)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("run_dir",type=Path)
    ap.add_argument("--h0-mm",type=float,default=180)
    args=ap.parse_args()
    if args.h0_mm<=0: ap.error("h0 must be positive")
    # Existing mount YAML specifies camera X -> body right,
    # camera Y -> body backward, optical Z -> body down (FRD).
    mount=np.array([[0,-1,0],[1,0,0],[0,0,1]],dtype=float)
    path=args.run_dir/"visual_z_lk_pairs.csv"
    out=args.run_dir/"visual_z_attitude_derotation.csv"
    n=0; total=0; cumulative=0.; first_t=None
    records=[]
    with path.open(newline="") as f,out.open("w",newline="") as o:
        reader=csv.DictReader(f)
        fields=["frame","time_s","pairs","inliers","rms_norm","logscale",
                "height_proxy_mm"]
        writer=csv.DictWriter(o,fieldnames=fields);writer.writeheader()
        current=None;group=[]
        def process(frame,group):
            nonlocal n,cumulative,first_t
            total_result=evaluate(group,mount)
            if total_result is None:return
            logscale,rms,inliers,points=total_result
            cumulative+=logscale
            t=int(group[0]["t1_ns"])*1e-9
            if first_t is None:first_t=t
            height=args.h0_mm*math.exp(-cumulative)
            writer.writerow(dict(frame=frame,time_s=t-first_t,pairs=points,
                                 inliers=inliers,rms_norm=rms,
                                 logscale=logscale,height_proxy_mm=height))
            records.append((t-first_t,height,rms,logscale))
            n+=1
        for row in reader:
            total+=1
            frame=int(row["frame"])
            if current is not None and frame!=current:
                process(current,group);group=[]
            current=frame;group.append(row)
        if group:process(current,group)
    print(f"INPUT_ROWS={total} VALID_FRAMES={n} OUTPUT={out}")
    if not records:return
    print(f"FINAL_PROXY_MM={records[-1][1]:.2f} MAX_PROXY_MM={max(x[1] for x in records):.2f}")
    print(f"MIN_PROXY_MM={min(x[1] for x in records):.2f}")
    for sec in (8,12,16,20,24,28,32,36,40):
        closest=min(records,key=lambda x:abs(x[0]-sec))
        print(f"t={closest[0]:.2f}s derot_proxy={closest[1]:.2f}mm")
    print("WARNING: zero distortion assumed; mount from OV9281 YAML; OV5647 not calibrated.")
    print("ATTITUDE timestamps may be imperfect; proxy is NOT validated height.")
    print("No runtime, WORKED5, or FC changes.")


if __name__=="__main__":
    main()
