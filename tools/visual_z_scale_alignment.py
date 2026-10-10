#!/usr/bin/env python3
"""Read-only, frame-aligned Visual-Z vs homography diagnostic."""
import argparse
import csv
import math
from pathlib import Path

import numpy as np


def number(row, key):
    try:
        v = float(row[key])
        return v if math.isfinite(v) else None
    except (KeyError, ValueError, TypeError):
        return None


def correlation(rows, x, y):
    a = np.array([(r[x], r[y]) for r in rows
                  if math.isfinite(r[x]) and math.isfinite(r[y])])
    if len(a) < 5 or np.std(a[:, 0]) < 1e-12 or np.std(a[:, 1]) < 1e-12:
        return float("nan")
    return float(np.corrcoef(a[:, 0], a[:, 1])[0, 1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--h0-mm", type=float, default=180.0)
    ap.add_argument("--bucket-s", type=float, default=4.0)
    args = ap.parse_args()
    if args.h0_mm <= 0 or args.bucket_s <= 0:
        ap.error("height and bucket must be positive")
    geom_path = args.run_dir / "visual_z_lk_geometry.csv"
    vz_path = args.run_dir / "visual_z_rotation_shadow.csv"
    with geom_path.open(newline="") as f:
        geom = {int(r["frame"]): r for r in csv.DictReader(f)}
    rows = []
    total = 0
    with vz_path.open(newline="") as f:
        for r in csv.DictReader(f):
            total += 1
            dt = number(r, "dt_s")
            rate = number(r, "raw_scale_rate")
            if number(r, "raw_valid") != 1 or dt is None or not 0 < dt < 0.2 or rate is None:
                continue
            g = geom.get(int(r["frame"]))
            if g is None:
                continue
            scale = number(g, "local_scale")
            if scale is None or scale <= 0:
                continue
            # Both are per-frame log-scale changes. Height uses the opposite sign.
            raw_dlog = rate * dt
            hom_dlog = math.log(scale)
            rows.append(dict(frame=int(r["frame"]), dt=dt, raw=raw_dlog,
                             hom=hom_dlog, shift=float(g["normalized_shift"]),
                             proj=float(g["projective_norm"]),
                             anis=float(g["anisotropy"]),
                             rms=float(g["rms_px"])))
    if not rows:
        ap.error("no matched valid frames")
    print(f"ROWS_VZ={total} HOMOGRAPHIES={len(geom)} MATCHED={len(rows)}")
    print("Correlation of PER-FRAME log-scale changes:")
    for key in ("hom", "shift", "proj", "anis", "rms"):
        print(f"  raw_vs_{key}={correlation(rows, 'raw', key):.5f}")
    t = 0.0
    raw_cum = hom_cum = 0.0
    bucket = 0
    output = []
    for r in rows:
        t += r["dt"]
        raw_cum += r["raw"]
        hom_cum += r["hom"]
        raw_mm = args.h0_mm * math.exp(-raw_cum)
        hom_mm = args.h0_mm * math.exp(-hom_cum)
        r.update(time_s=t, raw_mm=raw_mm, hom_mm=hom_mm,
                 raw_dlog_cum=raw_cum, hom_dlog_cum=hom_cum)
        output.append(r)
        if t >= bucket * args.bucket_s:
            print(f"t={t:6.2f}s raw={raw_mm:8.2f}mm homography_proxy={hom_mm:8.2f}mm "
                  f"shift={r['shift']:.5f} proj={r['proj']:.5f}")
            bucket += 1
    print(f"FINAL_RAW_MM={output[-1]['raw_mm']:.2f} "
          f"FINAL_HOMOGRAPHY_PROXY_MM={output[-1]['hom_mm']:.2f}")
    print(f"MAX_RAW_MM={max(r['raw_mm'] for r in output):.2f} "
          f"MAX_HOMOGRAPHY_PROXY_MM={max(r['hom_mm'] for r in output):.2f}")
    print("Homography proxy is NOT metric height; perspective/rotation may confound scale.")
    print("Correlations do not establish causality; no runtime or FC changes.")


if __name__ == "__main__":
    main()
