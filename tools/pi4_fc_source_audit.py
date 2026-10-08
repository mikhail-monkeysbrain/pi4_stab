#!/usr/bin/env python3
"""Inspect FC timing fields and MAVLink heartbeat source distribution. RX only."""
import argparse
import csv
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    sources = defaultdict(Counter)
    time_fields = defaultdict(Counter)
    heartbeats = defaultdict(list)
    with args.csv.open(newline="") as f:
        for row in csv.DictReader(f):
            kind = row.get("msg_type", "")
            src = (row.get("src_system", ""), row.get("src_component", ""))
            sources[kind][src] += 1
            for key in ("fc_time_boot_ms", "fc_time_usec"):
                value = row.get(key, "")
                if value not in ("", None):
                    time_fields[kind][key] += 1
            if kind == "HEARTBEAT":
                try:
                    heartbeats[src].append(int(row["recv_mono_ns"]))
                except (ValueError, KeyError):
                    pass
    for kind in sorted(sources):
        for src, count in sorted(sources[kind].items()):
            print(f"FC_SOURCE type={kind} sys={src[0]} comp={src[1]} rows={count} "
                  f"boot_ms_present={time_fields[kind]['fc_time_boot_ms']} "
                  f"time_usec_present={time_fields[kind]['fc_time_usec']}")
    for src, times in sorted(heartbeats.items()):
        gaps = [(b-a)/1e6 for a, b in zip(times, times[1:])]
        print(f"HEARTBEAT_SOURCE sys={src[0]} comp={src[1]} rows={len(times)} "
              f"median_gap_ms={median(gaps) if gaps else float('nan'):.3f} "
              f"max_gap_ms={max(gaps) if gaps else float('nan'):.3f}")
    print("NOTE: FC timestamp fields absent means clock offset/drift cannot be inferred from this CSV.")


if __name__ == "__main__":
    main()
