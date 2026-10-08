#!/usr/bin/env python3
"""Diagnostic: concurrently consume OV5647 YUV frames and incremental metadata.
Count alignment only; order correspondence is a hypothesis, NOT verified identity.
"""
import argparse
import json
import os
import select
import signal
import subprocess
import tempfile
import time
from pathlib import Path

FRAME_SIZE=640*480*3//2

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--seconds",type=float,default=6)
    args=p.parse_args()
    with tempfile.TemporaryDirectory(prefix="ov5647_dual_") as td:
        metadata=Path(td)/"metadata.json"
        cmd=["rpicam-vid","--camera","0","--width","640","--height","480",
             "--framerate","60","--codec","yuv420","--nopreview",
             "--timeout","0","--output","-",
             "--metadata",str(metadata),"--metadata-format","json"]
        proc=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,
                              bufsize=0)
        fd=proc.stdout.fileno()
        os.set_blocking(fd,False)
        decoder=json.JSONDecoder()
        pending=""
        consumed=0
        video_bytes=0
        frames=0
        meta_count=0
        meta_first=None
        frame_first=None
        last_ts=None
        invalid_ts=0
        nonmono=0
        max_abs_count_delta=0
        start=time.monotonic()
        next_report=start+1
        try:
            while time.monotonic()-start<args.seconds and proc.poll() is None:
                ready,_,_=select.select([fd],[],[],0.03)
                if ready:
                    try:
                        chunk=os.read(fd,1024*1024)
                    except BlockingIOError:
                        chunk=b""
                    if chunk:
                        video_bytes+=len(chunk)
                        new_frames=video_bytes//FRAME_SIZE-frames
                        if new_frames>0 and frame_first is None:
                            frame_first=time.monotonic()-start
                        frames+=new_frames
                if metadata.exists():
                    with metadata.open("r",encoding="utf-8") as f:
                        f.seek(consumed)
                        pending+=f.read()
                        consumed=f.tell()
                    while True:
                        pending=pending.lstrip(" \n\r\t,[]")
                        if not pending:
                            break
                        try:
                            obj,end=decoder.raw_decode(pending)
                        except json.JSONDecodeError:
                            break
                        pending=pending[end:]
                        if not isinstance(obj,dict):
                            continue
                        meta_count+=1
                        if meta_first is None:
                            meta_first=time.monotonic()-start
                        ts=obj.get("SensorTimestamp")
                        if not isinstance(ts,(int,float)):
                            invalid_ts+=1
                        else:
                            if last_ts is not None and ts<=last_ts:
                                nonmono+=1
                            last_ts=ts
                delta=frames-meta_count
                max_abs_count_delta=max(max_abs_count_delta,abs(delta))
                if time.monotonic()>=next_report:
                    print(f"DUAL_LIVE t={time.monotonic()-start:.2f}s frames={frames} "
                          f"metadata={meta_count} count_delta={delta}",flush=True)
                    next_report+=1
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
            # Keep draining stdout to avoid blocking camera shutdown on a full pipe.
            deadline=time.monotonic()+5
            while proc.poll() is None and time.monotonic()<deadline:
                ready,_,_=select.select([fd],[],[],0.1)
                if ready:
                    try:
                        chunk=os.read(fd,1024*1024)
                    except BlockingIOError:
                        chunk=b""
                    if chunk:
                        video_bytes+=len(chunk)
            if proc.poll() is None:
                proc.kill()
            proc.wait()
        try:
            final=json.loads(metadata.read_text())
            final_meta=len(final) if isinstance(final,list) else -1
        except (OSError,json.JSONDecodeError):
            final_meta=-1
        print(f"DUAL_FINAL frames_during={frames} metadata_during={meta_count} "
              f"final_metadata={final_meta} video_bytes={video_bytes} "
              f"partial_frame_bytes={video_bytes%FRAME_SIZE} "
              f"first_frame_s={frame_first} first_meta_s={meta_first} "
              f"max_abs_count_delta={max_abs_count_delta} "
              f"missing_ts={invalid_ts} nonmonotonic_ts={nonmono} rc={proc.returncode}")
        print("NOTE no_per_frame_timestamp_identity_claimed; counts can differ during polling")
        return 0 if frames>0 and meta_count>0 and invalid_ts==0 and nonmono==0 else 1

if __name__=="__main__":
    raise SystemExit(main())
