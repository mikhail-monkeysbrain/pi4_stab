#!/usr/bin/env python3
"""Pi4 read-only FC MAVLink capture for OV5647 integration.
No MAVLink transmit, no flow publication, no motor control.
FC time and RPi receive time are recorded separately; NOT synchronised.
"""
import argparse
import csv
import os
import signal
import time
from pathlib import Path

from pymavlink import mavutil

FIELDS = [
    "recv_mono_ns", "msg_type", "fc_time_boot_ms", "fc_time_usec",
    "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed",
    "xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro",
    "xmag", "ymag", "zmag",
    "system_status", "base_mode", "custom_mode", "src_system", "src_component",
]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--out", default="")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    output = Path(args.out or f"pi4_fc_readonly_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    # No heartbeat or stream-rate requests are sent: strict RX-only.
    conn = mavutil.mavlink_connection(args.port, baud=args.baud, autoreconnect=False)
    counts = {}
    start = time.monotonic()
    try:
        with output.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=FIELDS)
            writer.writeheader()
            while time.monotonic() - start < args.seconds:
                msg = conn.recv_match(blocking=False)
                if msg is None:
                    time.sleep(0.002)
                    continue
                kind = msg.get_type()
                if kind not in ("HEARTBEAT", "ATTITUDE", "RAW_IMU", "SCALED_IMU", "HIGHRES_IMU"):
                    continue
                recv_ns = time.monotonic_ns()
                row = {k: getattr(msg, k, "") for k in FIELDS}
                # Preserve native MAVLink clock fields under stable CSV names.
                row["fc_time_boot_ms"] = getattr(msg, "time_boot_ms", "")
                row["fc_time_usec"] = getattr(msg, "time_usec", "")
                row["recv_mono_ns"] = recv_ns
                row["msg_type"] = kind
                row["src_system"] = msg.get_srcSystem()
                row["src_component"] = msg.get_srcComponent()
                writer.writerow(row)
                counts[kind] = counts.get(kind, 0) + 1
    finally:
        conn.close()
    print(f"FC_READONLY out={output} elapsed_s={time.monotonic()-start:.2f} "
          f"heartbeat={counts.get('HEARTBEAT',0)} "
          f"attitude={counts.get('ATTITUDE',0)} "
          f"raw_imu={counts.get('RAW_IMU',0)} "
          f"scaled_imu={counts.get('SCALED_IMU',0)} "
          f"highres_imu={counts.get('HIGHRES_IMU',0)} "
          "clock_sync=UNVERIFIED tx=0")
    return 0 if counts.get("HEARTBEAT", 0) else 1

if __name__ == "__main__":
    raise SystemExit(main())
