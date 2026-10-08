#!/usr/bin/env python3
"""Estimate FC boot-clock / Pi receive-clock rate from ATTITUDE and SCALED_IMU.

RX-only offline analysis. Receive latency is unknown: slope is not a validated
sensor-time synchronization or camera/IMU extrinsic time offset.
"""
import argparse
import csv
from pathlib import Path
from statistics import median


def fit(rows):
    if len(rows) < 20:
        return None
    x0, y0 = rows[0]
    xs = [(x-x0)*1e-9 for x, _ in rows]
    ys = [(y-y0)*1e-9 for _, y in rows]
    mx, my = sum(xs)/len(xs), sum(ys)/len(ys)
    var = sum((x-mx)**2 for x in xs)
    if var <= 0:
        return None
    slope = sum((x-mx)*(y-my) for x,y in zip(xs,ys))/var
    offset = my-slope*mx
    residual_ms = [(y-(offset+slope*x))*1000 for x,y in zip(xs,ys)]
    abs_res = sorted(abs(r) for r in residual_ms)
    return slope, median(residual_ms), abs_res[round(.95*(len(abs_res)-1))], max(abs_res)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    args = ap.parse_args()
    data = {"ATTITUDE": [], "SCALED_IMU": []}
    raw_usec = []
    with args.csv.open(newline="") as f:
        for row in csv.DictReader(f):
            kind = row.get("msg_type")
            try:
                rx = int(row["recv_mono_ns"])
                if kind in data and row.get("fc_time_boot_ms"):
                    data[kind].append((rx, int(float(row["fc_time_boot_ms"])*1e6)))
                if kind == "RAW_IMU" and row.get("fc_time_usec"):
                    raw_usec.append(int(float(row["fc_time_usec"])))
            except (ValueError, KeyError):
                continue
    for kind, rows in data.items():
        print(f"CLOCK_FIT type={kind} rows={len(rows)}")
        for label, subset in (("full", rows),
                              ("first_half", rows[:len(rows)//2]),
                              ("second_half", rows[len(rows)//2:])):
            result = fit(subset)
            if result is None:
                print(f"  window={label} insufficient_data")
                continue
            slope, med, p95, worst = result
            print(f"  window={label} slope_fc_over_rx={slope:.9f} "
                  f"rate_delta_ppm={(slope-1)*1e6:.1f} "
                  f"residual_abs_p95_ms={p95:.3f} residual_abs_max_ms={worst:.3f}")
    if raw_usec:
        print(f"RAW_IMU_TIME rows={len(raw_usec)} min_usec={min(raw_usec)} "
              f"max_usec={max(raw_usec)} "
              f"nonmonotonic={sum(b<=a for a,b in zip(raw_usec,raw_usec[1:]))}")
    print("LIMITATION: UART receive timestamps are not exposure or IMU sample timestamps.")


if __name__ == "__main__":
    main()
