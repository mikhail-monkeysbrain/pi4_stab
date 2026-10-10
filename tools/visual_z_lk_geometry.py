#!/usr/bin/env python3
"""Offline LK planar geometry diagnostic; never changes runtime or FC.

Each point_id is local to its frame, not a persistent track ID.
"""
import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np


def analyze(frame, rows, reproj):
    if len(rows) < 20:
        return None
    p = np.asarray([[float(r["x0_px"]), float(r["y0_px"])] for r in rows], dtype=np.float64)
    q = np.asarray([[float(r["x1_px"]), float(r["y1_px"])] for r in rows], dtype=np.float64)
    good = np.isfinite(p).all(axis=1) & np.isfinite(q).all(axis=1)
    p, q = p[good], q[good]
    if len(p) < 20:
        return None
    H, mask = cv2.findHomography(p, q, cv2.RANSAC, reproj)
    if H is None or mask is None or abs(H[2, 2]) < 1e-12:
        return None
    H = H / H[2, 2]
    inliers = mask.ravel().astype(bool)
    if inliers.sum() < 15:
        return None
    predicted = cv2.perspectiveTransform(p[inliers].reshape(-1, 1, 2), H).reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((predicted - q[inliers]) ** 2, axis=1))))
    fx, fy, cx, cy = (float(rows[0][k]) for k in ("fx", "fy", "cx", "cy"))
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    if fx <= 0 or fy <= 0:
        return None
    A = np.linalg.inv(K) @ H @ K
    A /= A[2, 2]
    # Local Jacobian at principal ray: derivative of projective mapping.
    jac = A[:2, :2] - np.outer(A[:2, 2], A[2, :2])
    svals = np.linalg.svd(jac, compute_uv=False)
    det = float(np.linalg.det(jac))
    scale = math.sqrt(det) if det > 0 else float("nan")
    anis = float(svals[0] / svals[-1]) if svals[-1] > 1e-12 else float("inf")
    proj = float(np.linalg.norm(A[2, :2]))
    translation = float(np.linalg.norm(A[:2, 2]))
    t0, t1 = int(rows[0]["t0_ns"]), int(rows[0]["t1_ns"])
    return dict(frame=frame, t0_ns=t0, t1_ns=t1, pairs=len(p),
                inliers=int(inliers.sum()), inlier_ratio=float(inliers.mean()),
                rms_px=rms, local_scale=scale, anisotropy=anis,
                projective_norm=proj, normalized_shift=translation,
                h20=float(A[2, 0]), h21=float(A[2, 1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--ransac-px", type=float, default=2.0)
    args = ap.parse_args()
    if args.ransac_px <= 0:
        ap.error("--ransac-px must be positive")
    src = args.run_dir / "visual_z_lk_pairs.csv"
    dst = args.run_dir / "visual_z_lk_geometry.csv"
    fields = ["frame", "t0_ns", "t1_ns", "pairs", "inliers", "inlier_ratio",
              "rms_px", "local_scale", "anisotropy", "projective_norm",
              "normalized_shift", "h20", "h21"]
    count = 0
    # Stream one frame at a time: 500k+ rows need not fit in memory.
    with src.open(newline="") as f, dst.open("w", newline="") as out:
        reader = csv.DictReader(f)
        required = {"frame", "t0_ns", "t1_ns", "x0_px", "y0_px",
                    "x1_px", "y1_px", "fx", "fy", "cx", "cy"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            ap.error("missing columns: " + ", ".join(sorted(missing)))
        writer = csv.DictWriter(out, fieldnames=fields)
        writer.writeheader()
        current = None
        group = []
        for r in reader:
            frame = int(r["frame"])
            if current is not None and frame != current:
                result = analyze(current, group, args.ransac_px)
                if result:
                    writer.writerow(result)
                    count += 1
                group = []
            current = frame
            group.append(r)
        if current is not None:
            result = analyze(current, group, args.ransac_px)
            if result:
                writer.writerow(result)
                count += 1
    print(f"INPUT={src}")
    print(f"OUTPUT={dst}")
    print(f"VALID_HOMOGRAPHIES={count}")
    if count == 0:
        print("No valid homographies; check LK data.")
        return
    with dst.open(newline="") as f:
        rows = list(csv.DictReader(f))
    for key in ("inlier_ratio", "rms_px", "local_scale", "anisotropy",
                "projective_norm", "normalized_shift"):
        values = np.asarray([float(r[key]) for r in rows], dtype=float)
        values = values[np.isfinite(values)]
        if len(values):
            print(f"{key}: median={np.median(values):.6g} p95={np.percentile(values,95):.6g} max={np.max(values):.6g}")
    print("Homography scale is NOT an altitude estimate.")
    print("Planar translation, plane tilt, rotation and height can be ambiguous.")


if __name__ == "__main__":
    main()
