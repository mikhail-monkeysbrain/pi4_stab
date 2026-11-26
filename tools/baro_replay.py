#!/usr/bin/env python3

import argparse
import csv
import math
import statistics


def analyze(path):
    pressures = []
    ranges = []
    channels = []

    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                p = float(row["pressure_hpa"])
                if math.isfinite(p) and p > 0:
                    pressures.append(p)
            except (KeyError, ValueError):
                pass

            try:
                h = float(row["luna_m"])
                if math.isfinite(h) and h > 0:
                    ranges.append(h)
            except (KeyError, ValueError):
                pass

            try:
                channels.append(int(float(row["rc6_us"])))
            except (KeyError, ValueError):
                pass

    for name, values in (
        ("Pressure, hPa", pressures),
        ("Range, m", ranges),
        ("RC6, us", channels),
    ):
        print(f"\n===== {name} =====")
        if not values:
            print("NO DATA")
            continue

        print(f"Samples: {len(values)}")
        print(f"Min:     {min(values):.6f}")
        print(f"Max:     {max(values):.6f}")
        print(f"Median:  {statistics.median(values):.6f}")
        print(f"Spread:  {max(values)-min(values):.6f}")


def selftest():
    assert statistics.median([1000, 1001, 1002]) == 1001
    assert math.isfinite(1013.25)
    print("PASS: diagnostic selftest")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", nargs="?")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        selftest()
    elif args.csv_file:
        analyze(args.csv_file)
    else:
        parser.print_help()
