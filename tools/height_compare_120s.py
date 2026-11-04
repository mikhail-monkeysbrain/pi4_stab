#!/usr/bin/env python3
import csv
import time
from pathlib import Path

ROOT = Path.home() / "monkeysStab_runs"
OUT = Path.home() / "height_compare_120s.csv"
DURATION = 120.0

FIELDS = [
    "time_s",
    "mono_ns",
    "luna_m",
    "luna_age_ms",
    "range_to_fc_m",
    "range_sent",
    "ekf_z_ned",
    "ekf_age_ms",
    "ekf_local_valid",
]

def latest_log():
    files = list(ROOT.glob("*/optical_flow_mavlink.csv"))
    if not files:
        raise RuntimeError("Optical Flow CSV не найден")
    return max(files, key=lambda p: p.stat().st_mtime)

source = latest_log()
print("SOURCE:", source, flush=True)
print("OUTPUT:", OUT, flush=True)

start = time.monotonic()
rows = 0
luna_updates = 0
ekf_valid = 0
range_sent = 0

with source.open("r", newline="") as src, \
     OUT.open("w", newline="") as dst:

    header = src.readline()
    columns = next(csv.reader([header]))
    reader = csv.DictReader(src, fieldnames=columns)

    writer = csv.DictWriter(dst, fieldnames=FIELDS)
    writer.writeheader()

    # Начинаем с новых данных, не копируем старый лог.
    src.seek(0, 2)

    previous_luna_timestamp = None
    last_mono_ns = None

    while time.monotonic() - start < DURATION:
        pos = src.tell()
        line = src.readline()

        if not line:
            time.sleep(0.02)
            continue

        if not line.endswith("\n"):
            src.seek(pos)
            time.sleep(0.02)
            continue

        try:
            values = next(csv.reader([line]))
            if len(values) != len(columns):
                continue

            row = dict(zip(columns, values))
            mono_ns = int(row["mono_ns"])
            if last_mono_ns is not None and mono_ns <= last_mono_ns:
                continue
            last_mono_ns = mono_ns

            luna_age = float(row["luna_age_ms"])
            luna_timestamp = round(
                mono_ns / 1e9 - luna_age / 1000, 3
            )

            if (
                0 <= luna_age < 1000
                and float(row["luna_m"]) > 0
                and luna_timestamp != previous_luna_timestamp
            ):
                luna_updates += 1
                previous_luna_timestamp = luna_timestamp

            if row["ekf_local_valid"] == "1":
                ekf_valid += 1

            if row["range_sent"] == "1":
                range_sent += 1

            result = {
                k: row.get(k, "") for k in FIELDS
            }
            result["time_s"] = round(
                time.monotonic() - start, 6
            )

            writer.writerow(result)
            rows += 1

        except (ValueError, KeyError, csv.Error):
            continue

    dst.flush()

print("===== RESULT =====")
print("Duration:", round(time.monotonic() - start, 2), "s")
print("Rows:", rows)
print("Luna updates (estimated):", luna_updates)
print("EKF valid rows:", ekf_valid)
print("Range sent rows:", range_sent)
print("CSV:", OUT)
