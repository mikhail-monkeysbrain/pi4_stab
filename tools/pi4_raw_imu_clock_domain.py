#!/usr/bin/env python3
"""Offline compare RAW_IMU time_usec to ATTITUDE time_boot_ms via RX-time matching.

Uses nearest receive timestamp, NOT synchronized sampling. No FC TX.
"""
import argparse
import bisect
import csv
from pathlib import Path
from statistics import median


def p95(v):
    s=sorted(v)
    return s[round(.95*(len(s)-1))] if s else float("nan")


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("directory",type=Path)
    args=ap.parse_args()
    raw=[]; attitude=[]
    with (args.directory/"fc.csv").open(newline="") as f:
        for row in csv.DictReader(f):
            try:
                rx=int(row["recv_mono_ns"])
                if row["msg_type"]=="RAW_IMU" and row.get("fc_time_usec"):
                    raw.append((rx,int(row["fc_time_usec"])))
                elif row["msg_type"]=="ATTITUDE" and row.get("fc_time_boot_ms"):
                    attitude.append((rx,int(row["fc_time_boot_ms"])*1000))
            except (ValueError,KeyError):
                continue
    raw.sort(); attitude.sort()
    if len(raw)<20 or len(attitude)<20:
        raise SystemExit("ERROR: insufficient timestamped RAW_IMU or ATTITUDE messages")
    rx_times=[x for x,_ in attitude]
    differences=[]; rx_gaps=[]
    for rx,fc_usec in raw:
        idx=bisect.bisect_left(rx_times,rx)
        candidates=[attitude[j] for j in (idx-1,idx) if 0<=j<len(attitude)]
        other_rx,other_fc=min(candidates,key=lambda x:abs(x[0]-rx))
        differences.append((fc_usec-other_fc)/1000)
        rx_gaps.append(abs(rx-other_rx)/1e6)
    first,last=raw[0],raw[-1]
    duration_fc=(last[1]-first[1])/1e6
    duration_rx=(last[0]-first[0])/1e9
    print(f"RAW_IMU_DOMAIN rows={len(raw)} first_usec={first[1]} last_usec={last[1]} "
          f"fc_elapsed_s={duration_fc:.6f} rx_elapsed_s={duration_rx:.6f} "
          f"endpoint_rate_delta_ppm={(duration_fc/duration_rx-1)*1e6:.1f}")
    print(f"RAW_VS_ATTITUDE nearest_rx_pairs={len(differences)} "
          f"rx_separation_median_ms={median(rx_gaps):.3f} "
          f"rx_separation_p95_ms={p95(rx_gaps):.3f} "
          f"fc_timestamp_difference_median_ms={median(differences):.3f} "
          f"fc_timestamp_difference_p95_ms={p95(differences):.3f} "
          f"fc_timestamp_difference_min_ms={min(differences):.3f} "
          f"fc_timestamp_difference_max_ms={max(differences):.3f}")
    print("LIMITATION: nearest UART reception is not proof of simultaneous IMU and attitude sampling.")


if __name__=="__main__":
    main()
