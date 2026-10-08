#!/usr/bin/env python3
"""Joint OV5647 WORKED5 + barometer diagnostic, no flight outputs.

Camera and FC run as independent processes. No timestamp synchronization or
camera-to-ground metric scale is claimed. FC TX is limited to temporary
MAV_CMD_SET_MESSAGE_INTERVAL telemetry requests by pi4_baro_interval_probe.py.
"""
import argparse
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
        ap.error("--seconds must be 5..120")
    root = Path(__file__).resolve().parent.parent
    out = Path(args.out or f"/tmp/pi4_camera_baro_{time.strftime('%Y%m%d_%H%M%S')}")
    out.mkdir(parents=True, exist_ok=True)
    binary = out / "ov5647_libcamera_worked5_threaded"
    build = ["g++", "-std=c++17", "-O2", "-pthread", "-Isrc",
             "tools/ov5647_libcamera_worked5_threaded.cpp", "-o", str(binary)]
    build += subprocess.check_output(
        ["pkg-config", "--cflags", "--libs", "libcamera", "opencv4"],
        text=True).split()
    subprocess.run(build, cwd=root, check=True)
    baro_cmd = [args.python, str(root / "tools/pi4_baro_interval_probe.py"),
                "--port", args.port, "--baud", str(args.baud),
                "--seconds", str(args.seconds + 2)]
    cam_cmd = [str(binary), str(args.seconds), str(out / "camera_timestamps.csv"),
               str(out / "worked5_steps.csv")]
    with (out / "baro.log").open("w") as baro_log, (out / "camera.log").open("w") as cam_log:
        baro = subprocess.Popen(baro_cmd, cwd=root, stdout=baro_log,
                                stderr=subprocess.STDOUT)
        try:
            time.sleep(1)
            cam = subprocess.run(cam_cmd, cwd=root, stdout=cam_log,
                                 stderr=subprocess.STDOUT,
                                 timeout=args.seconds + 15)
            cam_rc = cam.returncode
        finally:
            try:
                baro_rc = baro.wait(timeout=args.seconds + 10)
            except subprocess.TimeoutExpired:
                baro.terminate()
                baro_rc = baro.wait(timeout=5)
    print("JOINT_CAMERA_BARO", f"dir={out}", f"camera_rc={cam_rc}",
          f"baro_rc={baro_rc}", "flight_output=OFF",
          "height=SYNTHETIC", "clock_sync=UNVERIFIED",
          "fc_tx=TELEMETRY_REQUEST_ONLY")
    for name, prefixes in (
        ("camera.log", ("LIBCAMERA_WORKED5", "REQUEST_FINAL", "REQUEST_PROBE_FAIL")),
        ("baro.log", ("TARGET", "ACK", "RESULT", "ERROR"))):
        for line in (out / name).read_text(errors="replace").splitlines():
            if line.startswith(prefixes):
                print(line)
    print("CAMERA_CSV", out / "camera_timestamps.csv")
    print("WORKED5_STEPS_CSV", out / "worked5_steps.csv")
    print("CAMERA_LOG", out / "camera.log")
    print("BARO_LOG", out / "baro.log")
    return 0 if cam_rc == 0 and baro_rc == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
