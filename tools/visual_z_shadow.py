#!/usr/bin/env python3
"""Offline Visual-Z shadow diagnostic. Does not change runtime or FC."""
import argparse
import csv
import math
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_path", nargs="?", type=Path)
    ap.add_argument("--h0", type=float, default=0.18, help="initial height in meters")
    ap.add_argument("--bucket", type=float, default=2.0, help="report interval seconds")
    args = ap.parse_args()
    p = args.csv_path
    if p is None:
        files = sorted((Path.home() / "monkeysStab_runs").rglob("optical_flow_mavlink.csv"),
                       key=lambda f: f.stat().st_mtime)
        if not files:
            ap.error("no logs found")
        p = files[-1]
    h = args.h0
    t = 0.0
    next_report = 0.0
    count = 0
    rejected = 0
    lo = hi = h
    print("LOG:", p)
    print("time_s,visual_z_mm,scale_rate_s_inv,inliers,inlier_ratio")
    with p.open(newline="") as f:
        for row in csv.DictReader(f):
            try:
                dt = float(row["dt_s"])
                rate = float(row["scale_rate"])
                valid = int(float(row["valid"]))
            except (KeyError, ValueError):
                rejected += 1
                continue
            if not (valid and 0 < dt < 0.2 and math.isfinite(rate)):
                rejected += 1
                continue
            h *= math.exp(-rate * dt)
            t += dt
            count += 1
            lo = min(lo, h)
            hi = max(hi, h)
            if t >= next_report:
                print(f"{t:.2f},{h*1000:.2f},{rate:.6f},{row.get('inliers','')},{row.get('inlier_ratio','')}")
                next_report = t + args.bucket
    print(f"VALID={count} REJECTED={rejected} VALID_DURATION_S={t:.2f}")
    print(f"MIN_MM={lo*1000:.2f} MAX_MM={hi*1000:.2f} FINAL_MM={h*1000:.2f}")
    print("NOTE: time_s accumulates valid dt only; gaps are omitted.")
    print("NOTE: initial height is assumed; absolute height is not observable from monocular scale alone.")

if __name__ == "__main__":
    main()
