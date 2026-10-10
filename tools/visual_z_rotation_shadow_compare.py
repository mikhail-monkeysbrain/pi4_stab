#!/usr/bin/env python3
"""Read-only comparison of raw and gyro-derotated Visual-Z on matched valid frames."""
import argparse
import csv
import math
from pathlib import Path

def val(row, key):
    try:
        x = float(row[key])
        return x if math.isfinite(x) else None
    except (KeyError, ValueError, TypeError):
        return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--h0", type=float, default=0.18)
    ap.add_argument("--bucket", type=float, default=2.0)
    args = ap.parse_args()
    if args.h0 <= 0 or args.bucket <= 0:
        ap.error("h0 and bucket must be positive")
    p = args.run_dir / "visual_z_rotation_shadow.csv"
    if not p.exists():
        ap.error(f"missing {p}; enable MONKEYS_PI4_VISUAL_Z_SHADOW=1 before runtime")
    h1 = h2 = args.h0
    t = 0.0
    total = accepted = invalid = 0
    next_print = 0.0
    print("time_s,raw_mm,derotated_mm,raw_rate,derotated_rate,affine_rms_px,points")
    with p.open(newline="") as f:
        for r in csv.DictReader(f):
            total += 1
            dt, rate = val(r, "dt_s"), val(r, "raw_scale_rate")
            fx, fy = val(r, "fx"), val(r, "fy")
            a00, a11 = val(r, "a00"), val(r, "a11")
            if (val(r, "raw_valid") != 1 or (val(r, "highres_valid") != 1 and val(r, "attitude_gyro_valid") != 1)
                    or val(r, "pixel_field_valid") != 1 or dt is None
                    or not 0 < dt < 0.2 or rate is None or fx is None
                    or fy is None or min(fx, fy) <= 0 or a00 is None or a11 is None):
                invalid += 1
                continue
            s = 0.5 * (a00/fx + a11/fy)
            h1 *= math.exp(-rate*dt)
            h2 *= math.exp(-s)
            t += dt
            accepted += 1
            if t >= next_print:
                print(f"{t:.2f},{h1*1000:.2f},{h2*1000:.2f},{rate:.6f},"
                      f"{s/dt:.6f},{r.get('affine_rms_px','')},{r.get('pixel_field_points','')}")
                next_print = t + args.bucket
    print(f"ROWS={total} MATCHED_VALID={accepted} REJECTED={invalid} "
          f"MATCHED_DURATION_S={t:.2f}")
    print(f"RAW_FINAL_MM={h1*1000:.2f} DEROTATED_FINAL_MM={h2*1000:.2f}")
    print("Both integrate matched valid frames only; skipped gaps are not reconstructed.")
    print("Derotated affine scale is experimental, not validated metric height.")
    print("No FC or runtime changes.")

if __name__ == "__main__":
    main()
