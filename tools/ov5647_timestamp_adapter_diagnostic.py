#!/usr/bin/env python3
"""OV5647 YUV+metadata diagnostic adapter, single rpicam-vid process.

WARNING: FIFO association is an UNVERIFIED hypothesis. No timestamps from
this tool may be used as flight-grade camera/IMU synchronization.
No MAVLink traffic, no WORKED5 changes.
"""
import argparse
import json
import os
import select
import signal
import subprocess
import tempfile
import time
from collections import deque
from pathlib import Path

W,H=640,480
FRAME_BYTES=W*H*3//2

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--seconds",type=float,default=8)
    p.add_argument("--max-pending",type=int,default=8)
    a=p.parse_args()
    if a.seconds<=0 or a.max_pending<2:
        p.error("seconds must be positive and max-pending >= 2")
    with tempfile.TemporaryDirectory(prefix="ov5647_adapter_") as td:
        metadata=Path(td)/"metadata.json"
        cmd=["rpicam-vid","--camera","0","--width",str(W),
             "--height",str(H),"--framerate","60","--codec","yuv420",
             "--nopreview","--timeout","0","--output","-",
             "--metadata",str(metadata),"--metadata-format","json"]
        proc=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,
                              bufsize=0)
        fd=proc.stdout.fileno()
        os.set_blocking(fd,False)
        raw=bytearray()
        pending_frames=deque()
        pending_meta=deque()
        json_buffer=""
        file_offset=0
        video_count=meta_count=paired=0
        bad_ts=bad_order=overflow=0
        last_ts=None
        first_pair_s=None
        max_video_pending=max_meta_pending=0
        max_pair_wait_ms=0.
        start=time.monotonic()
        next_print=start+1.
        try:
            while time.monotonic()-start<a.seconds and proc.poll() is None:
                ready,_,_=select.select([fd],[],[],0.015)
                if ready:
                    try:
                        chunk=os.read(fd,1024*1024)
                    except BlockingIOError:
                        chunk=b""
                    if chunk:
                        raw.extend(chunk)
                        while len(raw)>=FRAME_BYTES:
                            # Do not retain full frame: test validates transport only.
                            del raw[:FRAME_BYTES]
                            video_count+=1
                            pending_frames.append((video_count,time.monotonic_ns()))
                if metadata.exists():
                    with metadata.open("r",encoding="utf-8") as f:
                        f.seek(file_offset)
                        json_buffer+=f.read()
                        file_offset=f.tell()
                    while True:
                        json_buffer=json_buffer.lstrip(" \t\r\n,[]")
                        if not json_buffer:
                            break
                        try:
                            obj,end=json.JSONDecoder().raw_decode(json_buffer)
                        except json.JSONDecodeError:
                            break
                        json_buffer=json_buffer[end:]
                        if not isinstance(obj,dict):
                            continue
                        meta_count+=1
                        ts=obj.get("SensorTimestamp")
                        if not isinstance(ts,int) or ts<=0:
                            bad_ts+=1
                            continue
                        if last_ts is not None and ts<=last_ts:
                            bad_order+=1
                            continue
                        last_ts=ts
                        pending_meta.append((meta_count,ts))
                while pending_frames and pending_meta:
                    seq,received_ns=pending_frames.popleft()
                    meta_seq,ts=pending_meta.popleft()
                    # This equality only confirms FIFO counters, NOT identity.
                    if seq!=meta_seq:
                        bad_order+=1
                        continue
                    paired+=1
                    if first_pair_s is None:
                        first_pair_s=time.monotonic()-start
                    wait_ms=(time.monotonic_ns()-received_ns)/1e6
                    max_pair_wait_ms=max(max_pair_wait_ms,wait_ms)
                max_video_pending=max(max_video_pending,len(pending_frames))
                max_meta_pending=max(max_meta_pending,len(pending_meta))
                if len(pending_frames)>a.max_pending or len(pending_meta)>a.max_pending:
                    overflow+=1
                    print("ADAPTER_FAIL queue_overflow; synchronization invalid",flush=True)
                    break
                if time.monotonic()>=next_print:
                    print(f"ADAPTER_LIVE t={time.monotonic()-start:.2f} "
                          f"video={video_count} metadata={meta_count} "
                          f"fifo_pairs={paired} pending_video={len(pending_frames)} "
                          f"pending_meta={len(pending_meta)}",flush=True)
                    next_print+=1.
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
            # A full stdout pipe can prevent a graceful exit; drain briefly.
            deadline=time.monotonic()+3
            while proc.poll() is None and time.monotonic()<deadline:
                ready,_,_=select.select([fd],[],[],0.05)
                if ready:
                    try:
                        if not os.read(fd,1024*1024):
                            break
                    except BlockingIOError:
                        pass
            if proc.poll() is None:
                proc.kill()
            proc.wait()
        print(f"ADAPTER_FINAL video={video_count} metadata={meta_count} "
              f"fifo_pairs={paired} pending_video={len(pending_frames)} "
              f"pending_meta={len(pending_meta)} partial_video_bytes={len(raw)} "
              f"first_pair_s={first_pair_s} max_pair_wait_ms={max_pair_wait_ms:.3f} "
              f"max_video_pending={max_video_pending} "
              f"max_meta_pending={max_meta_pending} missing_or_bad_ts={bad_ts} "
              f"nonmonotonic_ts={bad_order} queue_overflow={overflow} "
              f"camera_rc={proc.returncode} association=UNVERIFIED")
        print("NO_FLIGHT_TIMESTAMP: FIFO count agreement does not prove frame identity")
        return 0 if paired>0 and not (bad_ts or bad_order or overflow) else 1

if __name__=="__main__":
    raise SystemExit(main())
