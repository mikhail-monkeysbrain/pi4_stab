#!/usr/bin/env python3
"""Single-owner MAVLink IMU capture with temporary interval requests.
Only MAV_CMD_SET_MESSAGE_INTERVAL is transmitted. No flight commands.
"""
import argparse
import csv
import time
from collections import Counter
from pathlib import Path
from pymavlink import mavutil

FIELDS = ["recv_mono_ns","msg_type","fc_time_boot_ms","fc_time_usec",
          "roll","pitch","yaw","rollspeed","pitchspeed","yawspeed",
          "xacc","yacc","zacc","xgyro","ygyro","zgyro",
          "xmag","ymag","zmag","src_system","src_component"]
TYPES = ("ATTITUDE","RAW_IMU","SCALED_IMU")


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--port",default="/dev/serial0")
    ap.add_argument("--baud",type=int,default=460800)
    ap.add_argument("--seconds",type=float,default=30)
    ap.add_argument("--out",required=True)
    args=ap.parse_args()
    conn=mavutil.mavlink_connection(args.port,baud=args.baud,
                                    source_system=254,source_component=191,
                                    autoreconnect=False)
    try:
        target=None
        until=time.monotonic()+8
        while time.monotonic()<until:
            hb=conn.recv_match(type="HEARTBEAT",blocking=False)
            if hb is None:
                time.sleep(0.01)
                continue
            if (hb.get_srcSystem()>0 and
                hb.get_srcComponent()==mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 and
                hb.autopilot!=mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                target=(hb.get_srcSystem(),hb.get_srcComponent())
                break
        if target is None:
            print("FC_IMU_REQUEST_FAIL no autopilot heartbeat",flush=True)
            return 2
        print(f"TARGET sys={target[0]} comp={target[1]}",flush=True)
        for kind in TYPES:
            msg_id=getattr(mavutil.mavlink,"MAVLINK_MSG_ID_"+kind)
            conn.mav.command_long_send(target[0],target[1],
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,
                msg_id,50000,0,0,0,0,0)
            print(f"REQUEST {kind} id={msg_id} interval_us=50000",flush=True)
            time.sleep(0.1)
        path=Path(args.out)
        path.parent.mkdir(parents=True,exist_ok=True)
        counts=Counter()
        ack=Counter()
        until=time.monotonic()+args.seconds
        with path.open("w",newline="") as f:
            writer=csv.DictWriter(f,fieldnames=FIELDS)
            writer.writeheader()
            while time.monotonic()<until:
                msg=conn.recv_match(blocking=False)
                if msg is None:
                    time.sleep(0.002)
                    continue
                if (msg.get_srcSystem(),msg.get_srcComponent())!=target:
                    continue
                kind=msg.get_type()
                if kind=="COMMAND_ACK" and getattr(msg,"command",None)==mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    ack[msg.result]+=1
                    print(f"ACK result={msg.result}",flush=True)
                if kind not in TYPES:
                    continue
                row={k:getattr(msg,k,"") for k in FIELDS}
                row.update(recv_mono_ns=time.monotonic_ns(),msg_type=kind,
                           fc_time_boot_ms=getattr(msg,"time_boot_ms",""),
                           fc_time_usec=getattr(msg,"time_usec",""),
                           src_system=msg.get_srcSystem(),
                           src_component=msg.get_srcComponent())
                writer.writerow(row)
                counts[kind]+=1
        print("FC_IMU_REQUEST",f"out={path}",
              *(f"{k.lower()}={counts[k]}" for k in TYPES),
              f"acks={sum(ack.values())}",
              "clock_sync=UNVERIFIED tx=INTERVAL_REQUEST_ONLY",flush=True)
        return 0 if all(counts[k]>0 for k in TYPES) else 1
    finally:
        conn.close()


if __name__=="__main__":
    raise SystemExit(main())
