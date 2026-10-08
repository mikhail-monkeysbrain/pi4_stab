#!/usr/bin/env python3
"""Diagnose Pi4 FC MAVLink RX, optionally request telemetry message intervals.
No arming, flight control, param changes or optical flow publication.
"""
import argparse
import collections
import time
from pymavlink import mavutil

MESSAGES = {
    "ATTITUDE": 30,
    "RAW_IMU": 27,
    "SCALED_IMU": 26,
    "HIGHRES_IMU": 105,
}

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--port",default="/dev/serial0")
    ap.add_argument("--baud",type=int,default=460800)
    ap.add_argument("--seconds",type=float,default=20)
    ap.add_argument("--target-system",type=int,default=1)
    ap.add_argument("--target-component",type=int,default=1)
    ap.add_argument("--request",action="store_true",
                    help="request ATTITUDE RAW_IMU SCALED_IMU HIGHRES_IMU at 20Hz")
    args=ap.parse_args()
    conn=mavutil.mavlink_connection(args.port,baud=args.baud,autoreconnect=False)
    counts=collections.Counter()
    src=collections.Counter()
    ack=collections.Counter()
    start=time.monotonic()
    requested=False
    target=None
    try:
        while time.monotonic()-start<args.seconds:
            m=conn.recv_match(blocking=False)
            if m is None:
                time.sleep(0.002)
                continue
            kind=m.get_type()
            if kind=="BAD_DATA":continue
            counts[kind]+=1
            src[(m.get_srcSystem(),m.get_srcComponent())]+=1
            if kind=="COMMAND_ACK":
                ack[(getattr(m,"command",None),getattr(m,"result",None))]+=1
            if (kind=="HEARTBEAT" and target is None and
                m.get_srcSystem()==args.target_system and
                m.get_srcComponent()==args.target_component):
                target=(m.get_srcSystem(),m.get_srcComponent())
            if args.request and target and not requested:
                requested=True
                for name,msgid in MESSAGES.items():
                    conn.mav.command_long_send(
                        target[0],target[1],
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0,msgid,50000,0,0,0,0,0)
                    print(f"REQUEST {name} id={msgid} interval_us=50000")
        print("MAVLINK_RX elapsed_s=%.2f requested=%d target=%s"%
              (time.monotonic()-start,int(requested),target))
        for name,n in counts.most_common():
            print(f"RX {name} count={n} rate_hz={n/args.seconds:.2f}")
        for source,n in src.most_common():
            print(f"SOURCE sys={source[0]} comp={source[1]} count={n}")
        for (cmd,result),n in ack.items():
            print(f"ACK command={cmd} result={result} count={n}")
    finally:
        conn.close()
    return 0 if (not args.request or requested) and counts.get("HEARTBEAT",0) else 1

if __name__=="__main__":
    raise SystemExit(main())
