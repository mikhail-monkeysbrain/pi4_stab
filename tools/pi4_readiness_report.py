#!/usr/bin/env python3
"""Read-only readiness summary from an existing Pi4 diagnostic directory."""
import argparse
import csv
from pathlib import Path


def count(path):
    with path.open(newline="") as handle:
        return sum(1 for _ in csv.DictReader(handle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    root = args.directory
    camera = root / "worked5_steps.csv"
    fc = root / "fc.csv"
    for path in (camera, fc):
        if not path.is_file():
            parser.error(f"missing {path}")
    print("PI4_READINESS_REPORT")
    print("worked5_steps=", count(camera))
    print("fc_messages=", count(fc))
    print("camera_and_imu_capture=OBSERVED")
    print("production_runtime=NOT_VERIFIED")
    print("optical_flow_fc_delivery=NOT_VERIFIED")
    print("true_agl=UNAVAILABLE")
    print("camera_calibration=PROVISIONAL")
    print("camera_imu_sync=UNVERIFIED")
    print("flight_readiness=BLOCKED")


if __name__ == "__main__":
    main()
