#!/usr/bin/env python3
import csv
import signal
import subprocess
import time
from pathlib import Path

HOME = Path.home()
ROOT = HOME / "monkeysStab_runs"
RECORDER = "/tmp/baro_ekf_120s"
BARO_FILE = HOME / "baro_ekf_120s.csv"
OUT = HOME / "height_3sources_60s.csv"
DURATION = 60.0

def latest_log():
    files = list(ROOT.glob("*/optical_flow_mavlink.csv"))
    if not files:
        raise RuntimeError("Optical Flow CSV не найден")
    return max(files, key=lambda p: p.stat().st_mtime)

def number(row, key, default=None):
    try:
        return float(row[key])
    except (ValueError, KeyError, TypeError):
        return default

if not Path(RECORDER).is_file():
    raise SystemExit("Не найден /tmp/baro_ekf_120s")

source = latest_log()
print("Optical Flow:", source, flush=True)

# Запоминаем время запуска и запускаем MAVLink-регистратор.
start_ns = time.monotonic_ns()

proc = subprocess.Popen(
    [RECORDER],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.STDOUT
)

rows = []
last_ns = None
luna_updates = 0
range_sent = 0
ekf_valid = 0
previous_luna_ns = None

try:
    with source.open("r", newline="") as f:
        header = f.readline()
        columns = next(csv.reader([header]))

        # Читаем только новые строки.
        f.seek(0, 2)

        while (time.monotonic_ns() - start_ns) < DURATION * 1e9:
            pos = f.tell()
            line = f.readline()

            if not line:
                time.sleep(0.01)
                continue

            if not line.endswith("\n"):
                f.seek(pos)
                time.sleep(0.01)
                continue

            try:
                values = next(csv.reader([line]))
                if len(values) != len(columns):
                    continue

                r = dict(zip(columns, values))
                mono_ns = int(r["mono_ns"])

                if last_ns is not None and mono_ns <= last_ns:
                    continue

                last_ns = mono_ns

                luna = number(r, "luna_m")
                age = number(r, "luna_age_ms")

                if luna is not None and luna > 0 and age is not None:
                    if 0 <= age < 1000:
                        measurement_ns = mono_ns - round(age * 1e6)

                        # Отсекаем повторения одного измерения.
                        if (
                            previous_luna_ns is None
                            or measurement_ns - previous_luna_ns > 100_000_000
                        ):
                            rows.append({
                                "source": "RANGE",
                                "mono_ns": measurement_ns,
                                "value": luna,
                                "aux": age,
                            })
                            luna_updates += 1
                            previous_luna_ns = measurement_ns

                if r.get("range_sent") == "1":
                    range_sent += 1

                if r.get("ekf_local_valid") == "1":
                    ekf_valid += 1

            except (ValueError, KeyError, csv.Error):
                continue

finally:
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

# Учитываем, что время в старом MAVLink CSV
# отсчитывается внутри регистратора, а не от mono_ns.
# Сохраняем отдельные временные шкалы без ложной
# миллисекундной синхронизации.
baro_count = 0
ekf_count = 0

if BARO_FILE.exists():
    with BARO_FILE.open(newline="") as f:
        for r in csv.DictReader(f):
            try:
                t = float(r["time_s"])
                kind = r["source"]

                if kind == "BARO":
                    value = float(r["pressure_hpa"])
                    aux = r.get("temperature_c", "")
                    baro_count += 1
                elif kind == "EKF":
                    value = float(r["ekf_z_m"])
                    aux = r.get("ekf_alt_m", "")
                    ekf_count += 1
                else:
                    continue

                rows.append({
                    "source": kind,
                    "mono_ns": "",
                    "value": value,
                    "aux": aux,
                    "time_s": t,
                })
            except (ValueError, KeyError):
                continue

with OUT.open("w", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=[
            "source", "time_s", "mono_ns", "value", "aux"
        ]
    )
    writer.writeheader()

    for r in rows:
        if r["source"] == "RANGE":
            r["time_s"] = (
                int(r["mono_ns"]) - start_ns
            ) / 1e9
        writer.writerow(r)

print("\n===== 60 SECOND TEST =====")
print("BARO:", baro_count)
print("EKF:", ekf_count)
print("RANGE updates:", luna_updates)
print("Range sent:", range_sent)
print("OF EKF valid rows:", ekf_valid)
print("OUTPUT:", OUT)
print("NOTE: MAVLink and RANGE timestamps have")
print("independent zero points; alignment is approximate.")
