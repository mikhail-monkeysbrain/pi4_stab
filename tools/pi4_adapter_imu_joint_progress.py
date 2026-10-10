#!/usr/bin/env python3
"""OV5647 adapter + frozen WORKED5 + FC IMU bench diagnostic.
Visible progress for long runs. FC TX limited to telemetry interval requests.
"""
import argparse
import csv
from pathlib import Path
import subprocess
import sys
import time


def progress(elapsed, total, camera, fc, previous_length=0):
    ratio = min(1.0, max(0.0, elapsed / total))
    cam = "RUN" if camera is None else str(camera)
    fc_status = "RUN" if fc is None else str(fc)
    suffix = f" {ratio * 100:3.0f}% {elapsed:.0f}/{total}s C:{cam} F:{fc_status}"
    import shutil
    columns = shutil.get_terminal_size(fallback=(60, 24)).columns
    width = max(5, min(24, columns - len(suffix) - 5))
    bar = "#" * round(width * ratio) + "-" * (width - round(width * ratio))
    line = f"[{bar}]{suffix}"
    # Never pad to a fixed 92 columns; avoid terminal line wrapping.
    line = line[:max(1, columns - 1)]
    sys.stdout.write("\r" + line + " " * max(0, previous_length - len(line)))
    sys.stdout.flush()
    return len(line)


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
    fc_cmd = [sys.executable, str(root / "tools/pi4_fc_imu_interval_capture.py"),
              "--port", args.port, "--baud", str(args.baud),
              "--seconds", str(args.seconds), "--out", str(out / "fc.csv")]
    print("START diagnostic; FC TX: temporary IMU interval requests only; no flow output", flush=True)
    with (out / "camera.log").open("w") as camera_log, (out / "fc.log").open("w") as fc_log:
        fc = subprocess.Popen(fc_cmd, cwd=root, stdout=fc_log, stderr=subprocess.STDOUT)
        camera = subprocess.Popen(camera_cmd, cwd=root, stdout=camera_log,
                                  stderr=subprocess.STDOUT)
        start = time.monotonic()
        previous_length = 0
        last_report = -1
        try:
            while True:
                elapsed = time.monotonic() - start
                cam_rc = camera.poll()
                fc_rc = fc.poll()
                if sys.stdout.isatty():
                    previous_length = progress(elapsed, args.seconds, cam_rc, fc_rc, previous_length)
                elif int(elapsed // 5) != last_report:
                    last_report = int(elapsed // 5)
                    print(f"PROGRESS {min(100, elapsed / args.seconds * 100):.0f}% elapsed={elapsed:.1f}s", flush=True)
                if cam_rc is not None and fc_rc is not None:
                    break
                if elapsed > args.seconds + 15:
                    raise TimeoutError("diagnostic exceeded expected duration")
                time.sleep(0.25)
            if sys.stdout.isatty():
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
            if sys.stdout.isatty():
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
          "calibration=PROVISIONAL", "fc_tx=INTERVAL_REQUEST_ONLY")
    for file, prefixes in (("camera.log", ("PI4_ADAPTER_", "PI4_ADAPTER_WORKED5_FAIL")),
                           ("fc.log", ("FC_IMU_REQUEST", "TARGET", "REQUEST", "ACK", "Traceback", "PermissionError"))):
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