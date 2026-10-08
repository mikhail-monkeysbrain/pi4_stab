#!/usr/bin/env python3
"""Analyze RX timing and FC boot clock drift from existing read-only CSV.

FC time and RPi receive time are NOT assumed synchronized.
"""
import argparse
import csv
from collections import defaultdict
from pathlib import Path
from statistics import median


def percentile(v, p):
    if not v:
        return float("nan")
    s = sorted(v)
    return s[round((len(s)-1)*p)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    args = ap.parse_args()
    grouped = defaultdict(list)
    with args.csv.open(newline="") as f:
        for row in csv.DictReader(f):
            kind = row.get("msg_type", "")
            try:
                rx = int(row["recv_mono_ns"])
            except (ValueError, KeyError):
                continue
            boot = row.get("fc_time_boot_ms", "")
            try:
                boot_ns = int(float(boot)*1e6) if boot != "" else None
            except ValueError:
                boot_ns = None
            grouped[kind].append((rx, boot_ns))
    for kind in ("HEARTBEAT", "ATTITUDE", "RAW_IMU", "SCALED_IMU"):
        rows = grouped.get(kind, [])
        if not rows:
            print(f"FC_TIMING type={kind} rows=0")
            continue
        rx = [r[0] for r in rows]
        gaps = [(b-a)/1e6 for a,b in zip(rx,rx[1:])]
        nonmono = sum(b <= a for a,b in zip(rx,rx[1:]))
        bootpairs = [(r,b) for r,b in rows if b is not None]
        result = (f"FC_TIMING type={kind} rows={len(rows)} "
                  f"rx_nonmonotonic={nonmono} "
                  f"rx_gap_median_ms={median(gaps) if gaps else float('nan'):.3f} "
                  f"rx_gap_p95_ms={percentile(gaps,.95):.3f} "
                  f"rx_gap_max_ms={max(gaps) if gaps else float('nan'):.3f}")
        if len(bootpairs)>1:
            elapsed_rx=bootpairs[-1][0]-bootpairs[0][0]
            elapsed_fc=bootpairs[-1][1]-bootpairs[0][1]
            fc_nonmono=sum(b2<=b1 for (_,b1),(_,b2) in zip(bootpairs,bootpairs[1:]))
            result+=(f" fc_boot_nonmonotonic={fc_nonmono} "
                     f"fc_boot_elapsed_s={elapsed_fc/1e9:.3f} "
                     f"rx_elapsed_s={elapsed_rx/1e9:.3f}")
            if elapsed_rx>0 and elapsed_fc>0:
                result+=f" endpoint_rate_delta_ppm={(elapsed_fc/elapsed_rx-1)*1e6:.1f}"
        print(result)
    print("NOTE: endpoint_rate_delta_ppm includes RX scheduling jitter; not a calibrated clock map.")


if __name__ == "__main__":
    main()
