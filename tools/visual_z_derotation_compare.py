#!/usr/bin/env python3
"""Compare existing gyro-derotated affine image scale to production Visual-Z.

Read-only: joins optical_flow_mavlink.csv and deltar_rotation_shadow.csv on frame.
Requires PIXEL_RESIDUAL_FIELD_V1 and valid HIGHRES rotation in the same run.
"""
import argparse
import csv
import math
from pathlib import Path


def num(r, k):
    try:
        v = float(r[k])
        return v if math.isfinite(v) else None
    except (KeyError, TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path, help="directory containing both CSV files")
    ap.add_argument("--fx", type=float, required=True, help="actual camera fx in pixels")
    ap.add_argument("--fy", type=float, required=True, help="actual camera fy in pixels")
    ap.add_argument("--h0", type=float, default=0.18)
    ap.add_argument("--bucket", type=float, default=2.0)
    args = ap.parse_args()
    if min(args.fx, args.fy, args.h0, args.bucket) <= 0:
        ap.error("fx, fy, h0 and bucket must be positive")
    a = args.run_dir / "optical_flow_mavlink.csv"
    b = args.run_dir / "deltar_rotation_shadow.csv"
    if not a.exists() or not b.exists():
        ap.error("both optical_flow_mavlink.csv and deltar_rotation_shadow.csv are required")
    with b.open(newline="") as f:
        shadows = {}
        for row in csv.DictReader(f):
            try:
                shadows[int(row["frame"])] = row
            except (KeyError, ValueError):
                continue
    t = 0.0
    hv1 = hv2 = args.h0
    joined = accepted = missing = invalid_rotation = 0
    next_report = 0.0
    print("time_s,v1_mm,derotated_affine_mm,delta_v1_mm,delta_derot_mm,"
          "raw_scale_rate,derot_scale_rate,affine_rms_px,points")
    with a.open(newline="") as f:
        for row in csv.DictReader(f):
            dt = num(row, "dt_s")
            rate = num(row, "scale_rate")
            if num(row, "valid") != 1 or dt is None or not 0 < dt < 0.2 or rate is None:
                continue
            try:
                sh = shadows.get(int(row["frame"]))
            except (KeyError, ValueError):
                sh = None
            if sh is None:
                missing += 1
                continue
            joined += 1
            if num(sh, "highres_valid") != 1 or num(sh, "pixel_field_valid") != 1:
                invalid_rotation += 1
                continue
            a00 = num(sh, "pixel_field_a00")
            a11 = num(sh, "pixel_field_a11")
            n = num(sh, "pixel_field_points")
            if a00 is None or a11 is None or n is None or n < 20:
                invalid_rotation += 1
                continue
            # The affine residual field is in pixel units vs normalized ray
            # coordinates. Divide the two diagonal gradients by fx/fy.
            s = 0.5 * (a00 / args.fx + a11 / args.fy)
            if not math.isfinite(s):
                invalid_rotation += 1
                continue
            old1, old2 = hv1, hv2
            hv1 *= math.exp(-rate * dt)
            hv2 *= math.exp(-s)
            t += dt
            accepted += 1
            if t >= next_report:
                print(f"{t:.2f},{hv1*1000:.2f},{hv2*1000:.2f},"
                      f"{(hv1-old1)*1000:.3f},{(hv2-old2)*1000:.3f},"
                      f"{rate:.6f},{s/dt:.6f},"
                      f"{num(sh,'pixel_field_affine_rms_px')},{int(n)}")
                next_report = t + args.bucket
    print(f"JOINED={joined} ACCEPTED={accepted} MISSING={missing} "
          f"INVALID_ROTATION_OR_FIELD={invalid_rotation}")
    print(f"ACCEPTED_DURATION_S={t:.2f} V1_FINAL_MM={hv1*1000:.2f} "
          f"DEROTATED_AFFINE_FINAL_MM={hv2*1000:.2f}")
    print("WARNING: height updates only on paired valid rows; missing intervals are omitted.")
    print("WARNING: affine trace is not yet a validated height estimate; "
          "translation/perspective leakage can remain after derotation.")
    print("No runtime, FC, or EKF3 changes.")


if __name__ == "__main__":
    main()
