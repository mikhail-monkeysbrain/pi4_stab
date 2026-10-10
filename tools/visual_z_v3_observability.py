#!/usr/bin/env python3
"""Visual-Z V3: offline affine consistency and observability audit.

This is NOT a metric height estimator. A planar scene admits a translation/
height ambiguity; affine consistency cannot certify absolute or relative Z.
"""
import argparse
import csv
import math
from pathlib import Path


def number(row, key):
    try:
        x = float(row[key])
        return x if math.isfinite(x) else None
    except (KeyError, TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--h0", type=float, default=0.18)
    ap.add_argument("--min-points", type=int, default=100)
    ap.add_argument("--max-rms-px", type=float, default=1.0)
    ap.add_argument("--max-anisotropy", type=float, default=0.5,
                    help="max ratio of traceless affine Frobenius norm to |trace/2|")
    args = ap.parse_args()
    if args.h0 <= 0 or args.min_points < 20 or args.max_rms_px <= 0:
        ap.error("invalid thresholds")
    path = args.run_dir / "visual_z_rotation_shadow.csv"
    h_raw = h_derot = args.h0
    n = accepted = suspect = invalid = 0
    max_raw = min_raw = max_derot = min_derot = args.h0
    reasons = {}
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        required = {"dt_s", "raw_valid", "raw_scale_rate", "highres_valid",
                    "attitude_gyro_valid", "pixel_field_valid",
                    "pixel_field_points", "fx", "fy", "a00", "a01",
                    "a10", "a11", "affine_rms_px"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            ap.error("missing CSV columns: " + ", ".join(sorted(missing)))
        for r in reader:
            n += 1
            dt = number(r, "dt_s")
            rate = number(r, "raw_scale_rate")
            fx, fy = number(r, "fx"), number(r, "fy")
            aa = [number(r, k) for k in ("a00", "a01", "a10", "a11")]
            points = number(r, "pixel_field_points")
            rms = number(r, "affine_rms_px")
            valid = (number(r, "raw_valid") == 1 and
                     (number(r, "highres_valid") == 1 or number(r, "attitude_gyro_valid") == 1) and
                     number(r, "pixel_field_valid") == 1 and
                     dt is not None and 0 < dt < 0.2 and rate is not None and
                     fx is not None and fx > 0 and fy is not None and fy > 0 and
                     all(v is not None for v in aa) and points is not None and
                     rms is not None)
            if not valid:
                invalid += 1
                continue
            a00, a01, a10, a11 = aa
            # Normalize the pixel displacement Jacobian into image ray units.
            j00, j01 = a00 / fx, a01 / fx
            j10, j11 = a10 / fy, a11 / fy
            isotropic = (j00 + j11) / 2
            anisotropic = math.sqrt(((j00-j11)/2)**2 +
                                    (j01*j01+j10*j10)/2)
            ratio = anisotropic / max(abs(isotropic), 1e-6)
            h_raw *= math.exp(-rate*dt)
            h_derot *= math.exp(-isotropic)
            max_raw, min_raw = max(max_raw, h_raw), min(min_raw, h_raw)
            max_derot, min_derot = max(max_derot, h_derot), min(min_derot, h_derot)
            flags = []
            if points < args.min_points: flags.append("few_points")
            if rms > args.max_rms_px: flags.append("high_rms")
            if ratio > args.max_anisotropy: flags.append("anisotropy")
            if flags:
                suspect += 1
                for reason in flags:
                    reasons[reason] = reasons.get(reason, 0) + 1
            else:
                accepted += 1
    print(f"RUN={args.run_dir.name}")
    print(f"ROWS={n} VALID={accepted+suspect} INVALID={invalid}")
    print(f"GEOMETRY_CONSISTENT={accepted} SUSPECT={suspect}")
    print("SUSPECT_REASONS=" + str(reasons))
    print(f"RAW_FINAL_MM={h_raw*1000:.2f} RAW_RANGE_MM={min_raw*1000:.2f}..{max_raw*1000:.2f}")
    print(f"DEROT_FINAL_MM={h_derot*1000:.2f} DEROT_RANGE_MM={min_derot*1000:.2f}..{max_derot*1000:.2f}")
    print("WARNING: GEOMETRY_CONSISTENT does NOT mean height observable.")
    print("Planar horizontal motion can mimic scale with low RMS and low anisotropy.")
    print("No FC TX, no WORKED5 changes.")


if __name__ == "__main__":
    main()
