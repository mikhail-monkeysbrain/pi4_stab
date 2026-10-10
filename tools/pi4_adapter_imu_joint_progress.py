#!/usr/bin/env python3
"""OV5647 adapter + frozen WORKED5 + FC IMU RX-only bench diagnostic.
Visible progress for every run longer than 9 seconds. No MAVLink TX.
"""
import argparse
import csv
from pathlib import Path
import subprocess
import sys
import time


def progress(elapsed, total, camera, fc):
    ratio = min(1.0, max(0.0, elapsed / total))
    width = 30
    filled = round(width * ratio)
    bar = "#" * filled + "-" * (width - filled)
    print(f"\rRUN [{bar}] {ratio * 100:5.1f}% "
          f"{elapsed:5.1f}/{total}s camera={'RUN' if camera is None else camera} "
          f"fc={'RUN' if fc is None else fc}", end="", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=30)
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    if not 5 <= args.seconds <= 120:
        ap.error("--seconds must be 5..120")
    root = Path(__file__).resolve().parent.parent
    out = Path(args.out or f"/tmp/pi4_adapter_imu_{time.strftime('%Y%m%d_%H%M%S')}")
    out.mkdir(parents=True, exist_ok=True)
    binary = out / "pi4_ov5647_adapter_worked5"
    build = ["g++", "-std=c++17", "-O2", "-pthread", "-Isrc",
             "tools/pi4_ov5647_adapter_worked5.cpp", "-o", str(binary)]
    build += subprocess.check_output(
        ["pkg-config", "--cflags", "--libs", "libcamera", "opencv4"],
        text=True).split()
    print("BUILD adapter + WORKED5", flush=True)
    subprocess.run(build, cwd=root, check=True)
    camera_cmd = [str(binary), str(args.seconds), str(out / "worked5_steps.csv")]
    fc_cmd = [sys.executable, str(root / "tools/pi4_fc_readonly_capture.py"),
              "--port", args.port, "--baud", str(args.baud),
              "--seconds", str(args.seconds), "--out", str(out / "fc.csv")]
    print("START diagnostic; FC is strictly RX-only; no flow output", flush=True)
    with (out / "camera.log").open("w") as camera_log, (out / "fc.log").open("w") as fc_log:
        fc = subprocess.Popen(fc_cmd, cwd=root, stdout=fc_log, stderr=subprocess.STDOUT)
        camera = subprocess.Popen(camera_cmd, cwd=root, stdout=camera_log,
                                  stderr=subprocess.STDOUT)
        start = time.monotonic()
        try:
            while True:
                elapsed = time.monotonic() - start
                cam_rc = camera.poll()
                fc_rc = fc.poll()
                progress(elapsed, args.seconds, cam_rc, fc_rc)
                if cam_rc is not None and fc_rc is not None:
                    break
                if elapsed > args.seconds + 15:
                    raise TimeoutError("diagnostic exceeded expected duration")
                time.sleep(0.25)
            print()
        finally:
            for proc in (camera, fc):
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
            print(flush=True)
    counts = {}
    fc_csv = out / "fc.csv"
    if fc_csv.exists():
        with fc_csv.open(newline="") as f:
            for row in csv.DictReader(f):
                kind = row["msg_type"]
                counts[kind] = counts.get(kind, 0) + 1
    print("JOINT_ADAPTER_IMU", f"dir={out}", f"camera_rc={camera.returncode}",
          f"fc_rc={fc.returncode}",
          *(f"{k.lower()}={counts.get(k, 0)}" for k in
            ("HEARTBEAT", "ATTITUDE", "RAW_IMU", "SCALED_IMU")),
          "clock_sync=UNVERIFIED", "height=SYNTHETIC",
          "calibration=PROVISIONAL", "no_fc_tx=1")
    for file, prefixes in (("camera.log", ("PI4_ADAPTER_", "PI4_ADAPTER_WORKED5_FAIL")),
                           ("fc.log", ("FC_READONLY", "Traceback", "PermissionError"))):
        for line in (out / file).read_text(errors="replace").splitlines():
            if line.startswith(prefixes):
                print(line)
    print("WORKED5_CSV", out / "worked5_steps.csv")
    print("FC_CSV", fc_csv)
    print("CAMERA_LOG", out / "camera.log")
    print("FC_LOG", out / "fc.log")
    return 0 if camera.returncode == 0 and fc.returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
