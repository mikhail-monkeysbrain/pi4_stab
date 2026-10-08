#!/usr/bin/env python3
"""Безопасная проверка CSI OV5647: только камера, без FC и EKF.

Читает YUV420 из rpicam-vid, выделяет Y-плоскость и измеряет FPS.
Не является источником точных timestamp для синхронизации с IMU.
"""
import argparse
import subprocess
import time
import sys

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seconds", type=float, default=15)
    p.add_argument("--fps", type=int, default=60)
    args = p.parse_args()
    w, h = 640, 480
    frame_bytes = w * h * 3 // 2
    cmd = ["rpicam-vid", "--camera", "0", "--width", str(w),
           "--height", str(h), "--framerate", str(args.fps),
           "--codec", "yuv420", "--nopreview", "--timeout", "0",
           "--output", "-"]
    print("CAMERA:", " ".join(cmd), flush=True)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            bufsize=0)
    count = 0
    t0 = time.monotonic()
    last = t0
    max_gap = 0.0
    try:
        while time.monotonic() - t0 < args.seconds:
            data = bytearray()
            while len(data) < frame_bytes:
                chunk = proc.stdout.read(frame_bytes - len(data))
                if not chunk:
                    raise RuntimeError("rpicam-vid stopped or returned incomplete frame")
                data.extend(chunk)
            now = time.monotonic()
            if count:
                max_gap = max(max_gap, now - last)
            last = now
            # First 640*480 bytes are the grayscale Y plane, suitable for cv::Mat.
            y_plane = memoryview(data)[:w*h]
            if len(y_plane) != w*h:
                raise RuntimeError("Invalid Y plane")
            count += 1
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
    duration = time.monotonic() - t0
    print(f"frames={count} elapsed_s={duration:.3f} fps={count/duration:.2f} max_read_gap_ms={max_gap*1000:.2f}")
    print("PASS: OV5647 YUV420 frame ingestion; NOT a WORKED5/IMU timing test")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"FAIL: {e}", file=sys.stderr)
        sys.exit(1)
