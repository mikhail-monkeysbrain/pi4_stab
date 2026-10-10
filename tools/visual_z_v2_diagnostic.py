#!/usr/bin/env python3
"""Visual-Z V2 diagnostic (offline only). Does not modify FC or runtime.

Compares apparent scale with XY optical flow and FC attitude in time bins.
Correlations are descriptive, not proof of causation. No new height estimator.
"""
import argparse
import csv
import math
from pathlib import Path


def number(row, key):
    try:
        x = float(row.get(key, ""))
        return x if math.isfinite(x) else None
    except (ValueError, TypeError):
        return None


def correlation(pairs):
    if len(pairs) < 3:
        return None
    xs, ys = zip(*pairs)
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    vx = sum((x - mx) ** 2 for x in xs)
    vy = sum((y - my) ** 2 for y in ys)
    if vx <= 1e-20 or vy <= 1e-20:
        return None
    return sum((x - mx) * (y - my) for x, y in pairs) / math.sqrt(vx * vy)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv_path", nargs="?", type=Path)
    ap.add_argument("--h0", type=float, default=0.18)
    ap.add_argument("--bucket", type=float, default=2.0)
    args = ap.parse_args()
    if args.h0 <= 0 or args.bucket <= 0:
        ap.error("--h0 and --bucket must be positive")
    p = args.csv_path
    if p is None:
        files = sorted((Path.home() / "monkeysStab_runs").rglob("optical_flow_mavlink.csv"),
                       key=lambda f: f.stat().st_mtime)
        if not files:
            ap.error("no optical_flow_mavlink.csv found")
        p = files[-1]
    h = args.h0
    valid = rejected = 0
    t = 0.0
    buckets = {}
    pairs = {k: [] for k in ("du_norm", "dv_norm", "flow_body_x", "flow_body_y",
                             "fc_roll", "fc_pitch", "fc_gyro_x", "fc_gyro_y")}
    with p.open(newline="") as f:
        reader = csv.DictReader(f)
        required = {"valid", "dt_s", "scale_rate"}
        if not required.issubset(reader.fieldnames or []):
            ap.error("CSV lacks columns: " + ", ".join(sorted(required - set(reader.fieldnames or []))))
        for row in reader:
            dt, rate, is_valid = (number(row, k) for k in ("dt_s", "scale_rate", "valid"))
            if is_valid != 1 or dt is None or not 0 < dt < 0.2 or rate is None:
                rejected += 1
                continue
            before = h
            h *= math.exp(-rate * dt)
            t += dt
            valid += 1
            b = int(t / args.bucket)
            d = buckets.setdefault(b, {"count": 0, "h0": before, "h1": h,
                                        "min": h, "max": h, "sum_rate": 0.0,
                                        "sum_abs_xy": 0.0, "n_xy": 0,
                                        "sum_roll": 0.0, "n_roll": 0,
                                        "sum_pitch": 0.0, "n_pitch": 0})
            d["count"] += 1
            d["h1"] = h
            d["min"] = min(d["min"], h)
            d["max"] = max(d["max"], h)
            d["sum_rate"] += rate
            x, y = number(row, "du_norm"), number(row, "dv_norm")
            if x is not None and y is not None:
                d["sum_abs_xy"] += math.hypot(x, y)
                d["n_xy"] += 1
            for k in ("fc_roll", "fc_pitch"):
                v = number(row, k)
                if v is not None:
                    d["sum_" + k[3:]] += v
                    d["n_" + k[3:]] += 1
            for k in pairs:
                v = number(row, k)
                if v is not None:
                    pairs[k].append((v, rate))
    print("LOG:", p)
    print("time_bin_s,visual_z_start_mm,visual_z_end_mm,delta_mm,min_mm,max_mm,"
          "mean_scale_rate,mean_xy_step_norm,mean_fc_roll,mean_fc_pitch,n")
    for i, d in sorted(buckets.items()):
        def avg(s, n):
            return f"{d[s] / d[n]:.6f}" if d[n] else "NA"
        print(f"{i * args.bucket:.1f}-{(i+1)*args.bucket:.1f},"
              f"{d['h0']*1000:.2f},{d['h1']*1000:.2f},"
              f"{(d['h1']-d['h0'])*1000:.2f},"
              f"{d['min']*1000:.2f},{d['max']*1000:.2f},"
              f"{d['sum_rate']/d['count']:.6f},"
              f"{avg('sum_abs_xy','n_xy')},{avg('sum_roll','n_roll')},"
              f"{avg('sum_pitch','n_pitch')},{d['count']}")
    print(f"VALID={valid} REJECTED={rejected} VALID_DURATION_S={t:.2f} FINAL_MM={h*1000:.2f}")
    print("CORRELATION_WITH_SCALE_RATE (per valid frame, unlagged):")
    for k, values in pairs.items():
        c = correlation(values)
        print(f"  {k}: {c:.4f} (n={len(values)})" if c is not None
              else f"  {k}: NA (n={len(values)})")
    print("NOTE: FC attitude units and synchronization must be checked before physical interpretation.")
    print("NOTE: time is cumulative accepted dt; gaps are omitted.")
    print("NOTE: correlation does not prove cause; this tool does not alter XY, FC or EKF3.")


if __name__ == "__main__":
    main()
