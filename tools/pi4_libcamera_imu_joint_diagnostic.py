#!/usr/bin/env python3
"""Concurrent direct-libcamera WORKED5 and FC MAVLink RX-only diagnostic.

Independent logs. SensorTimestamp and FC boot timestamps are NOT synchronized.
No MAVLink TX and no flight output.
"""
import argparse
import csv
from pathlib import Path
import subprocess
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=20)
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--python", default=sys.executable)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    if not 5 <= args.seconds <= 120:
        ap.error("--seconds must be between 5 and 120")
    root = Path(__file__).resolve().parent.parent
    out = Path(args.out or f"/tmp/pi4_libcamera_imu_{time.strftime('%Y%m%d_%H%M%S')}")
    out.mkdir(parents=True, exist_ok=True)
    binary = out / "ov5647_libcamera_worked5_threaded"
    build = ["g++", "-std=c++17", "-O2", "-pthread", "-Isrc",
             "tools/ov5647_libcamera_worked5_threaded.cpp", "-o", str(binary)]
    build += subprocess.check_output(
        ["pkg-config", "--cflags", "--libs", "libcamera", "opencv4"],
        text=True).split()
    print("BUILD", " ".join(build), flush=True)
    subprocess.run(build, cwd=root, check=True)

    fc_cmd = [args.python, str(root / "tools/pi4_fc_readonly_capture.py"),
              "--port", args.port, "--baud", str(args.baud),
              "--seconds", str(args.seconds + 2), "--out", str(out / "fc.csv")]
    camera_cmd = [str(binary), str(args.seconds)]
    fc_rc = -1
    camera_rc = -1
    with (out / "fc.log").open("w") as fc_log, (out / "camera.log").open("w") as camera_log:
        fc = subprocess.Popen(fc_cmd, cwd=root, stdout=fc_log, stderr=subprocess.STDOUT)
        try:
            time.sleep(1)
            camera = subprocess.run(camera_cmd, cwd=root, stdout=camera_log,
                                    stderr=subprocess.STDOUT,
                                    timeout=args.seconds + 15)
            camera_rc = camera.returncode
        finally:
            try:
                fc_rc = fc.wait(timeout=args.seconds + 10)
            except subprocess.TimeoutExpired:
                fc.terminate()
                try:
                    fc_rc = fc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    fc.kill()
                    fc_rc = fc.wait()

    counts = {}
    fc_csv = out / "fc.csv"
    if fc_csv.exists():
        with fc_csv.open(newline="") as f:
            for row in csv.DictReader(f):
                kind = row["msg_type"]
                counts[kind] = counts.get(kind, 0) + 1
    print("JOINT_LIBCAMERA_IMU", f"dir={out}", f"camera_rc={camera_rc}",
          f"fc_rc={fc_rc}", *(f"{k.lower()}={counts.get(k, 0)}"
                            for k in ("HEARTBEAT", "ATTITUDE", "RAW_IMU", "SCALED_IMU")),
          "clock_sync=UNVERIFIED", "no_fc_tx=1")
    print("CAMERA_LOG", out / "camera.log")
    print("FC_LOG", out / "fc.log")
    print("FC_CSV", fc_csv)
    if (out / "camera.log").exists():
        for line in (out / "camera.log").read_text(errors="replace").splitlines():
            if line.startswith(("LIBCAMERA_WORKED5", "REQUEST_FINAL", "REQUEST_PROBE_FAIL")):
                print(line)
    if (out / "fc.log").exists():
        for line in (out / "fc.log").read_text(errors="replace").splitlines():
            if line.startswith(("FC_READONLY", "Traceback", "PermissionError")):
                print(line)
    return 0 if camera_rc == 0 and fc_rc == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
