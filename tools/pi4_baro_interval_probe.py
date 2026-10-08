#!/usr/bin/env python3
"""Temporary MAVLink telemetry interval requests for barometer/altitude diagnosis.

Sends only MAV_CMD_SET_MESSAGE_INTERVAL. No arming, flight control, OF or
persistent parameter writes. Restoring prior stream rates is NOT guaranteed.
"""
import argparse
import time
from collections import Counter
from pymavlink import mavutil

MESSAGES = ("SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3",
            "ALTITUDE", "GLOBAL_POSITION_INT", "LOCAL_POSITION_NED", "VFR_HUD")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--seconds", type=float, default=20)
    args = ap.parse_args()
    if args.seconds <= 0: ap.error("--seconds must be positive")
    conn = mavutil.mavlink_connection(args.port, baud=args.baud,
                                     source_system=254, source_component=191,
                                     autoreconnect=False)
    try:
        hb = conn.wait_heartbeat(timeout=8)
        if hb is None:
            raise SystemExit("ERROR: FC heartbeat timeout")
        target_sys = conn.target_system
        target_comp = conn.target_component
        print(f"TARGET sys={target_sys} comp={target_comp}", flush=True)
        ids = {}
        for name in MESSAGES:
            mid = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_" + name, None)
            if mid is None:
                print(f"UNSUPPORTED_MESSAGE {name}", flush=True)
                continue
            ids[mid] = name
            conn.mav.command_long_send(
                target_sys, target_comp,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, mid, 200000, 0, 0, 0, 0, 0)
            print(f"REQUEST {name} id={mid} interval_us=200000", flush=True)
            time.sleep(0.1)
        counts = Counter()
        samples = {}
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            msg = conn.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.002)
                continue
            if msg.get_srcSystem() != target_sys or msg.get_srcComponent() != target_comp:
                continue
            kind = msg.get_type()
            if kind in MESSAGES:
                counts[kind] += 1
                if kind not in samples:
                    fields = ("press_abs", "temperature", "relative_alt", "alt",
                              "z", "altitude_relative", "altitude_local", "climb")
                    samples[kind] = {k: getattr(msg, k) for k in fields if hasattr(msg, k)}
            elif kind == "COMMAND_ACK":
                if getattr(msg, "command", None) == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    print(f"ACK result={msg.result}", flush=True)
        for name in MESSAGES:
            print(f"RESULT {name} count={counts[name]} sample={samples.get(name, {})}", flush=True)
        print("NOTE request changes telemetry intervals in FC runtime; no persistent parameter write.")
        print("NOTE pressure and EKF altitude do not measure camera-to-ground distance.")
    finally:
        conn.close()


if __name__ == "__main__":
    main()
