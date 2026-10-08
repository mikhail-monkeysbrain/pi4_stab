#!/usr/bin/env python3
"""SpeedyBee barometer and EKF altitude availability, strict MAVLink RX-only.

No MAVLink stream requests, no FC TX, no optical-flow publication.
Altitude estimates are NOT distance to ground.
"""
import argparse
import csv
import time
from pathlib import Path
from pymavlink import mavutil

KINDS = ("HEARTBEAT", "SCALED_PRESSURE", "SCALED_PRESSURE2",
         "SCALED_PRESSURE3", "ALTITUDE", "GLOBAL_POSITION_INT",
         "LOCAL_POSITION_NED", "VFR_HUD")
FIELDS = ("recv_mono_ns", "msg_type", "src_system", "src_component",
          "time_boot_ms", "time_usec", "press_abs", "press_diff",
          "temperature", "press_diff", "altitude_monotonic", "altitude_amsl",
          "altitude_local", "altitude_relative", "altitude_terrain",
          "bottom_clearance", "relative_alt", "alt", "z", "climb")


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--port",default="/dev/serial0")
    ap.add_argument("--baud",type=int,default=460800)
    ap.add_argument("--seconds",type=float,default=30)
    ap.add_argument("--out",default="")
    args=ap.parse_args()
    if args.seconds<=0: ap.error("--seconds must be positive")
    out=Path(args.out or f"/tmp/pi4_baro_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    out.parent.mkdir(parents=True,exist_ok=True)
    conn=mavutil.mavlink_connection(args.port,baud=args.baud,autoreconnect=False)
    counts={}
    start=time.monotonic()
    try:
        with out.open("w",newline="") as f:
            writer=csv.DictWriter(f,fieldnames=tuple(dict.fromkeys(FIELDS)))
            writer.writeheader()
            while time.monotonic()-start<args.seconds:
                msg=conn.recv_match(blocking=False)
                if msg is None:
                    time.sleep(.002)
                    continue
                kind=msg.get_type()
                if kind not in KINDS: continue
                row={key:getattr(msg,key,"") for key in FIELDS}
                row["recv_mono_ns"]=time.monotonic_ns()
                row["msg_type"]=kind
                row["src_system"]=msg.get_srcSystem()
                row["src_component"]=msg.get_srcComponent()
                writer.writerow(row)
                counts[kind]=counts.get(kind,0)+1
    finally:
        conn.close()
    print("BARO_RX_ONLY",f"out={out}",f"elapsed_s={time.monotonic()-start:.2f}",
          *(f"{k.lower()}={counts.get(k,0)}" for k in KINDS),"tx=0")
    if not any(counts.get(k,0) for k in KINDS if k!="HEARTBEAT"):
        print("BARO_NOTE no altitude/pressure messages received; RX-only cannot request streams")
    print("BARO_NOTE relative_alt and LOCAL_POSITION_NED.z are EKF estimates, not ground clearance")
    return 0 if counts.get("HEARTBEAT",0)>0 else 1


if __name__=="__main__":
    raise SystemExit(main())
