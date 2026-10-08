#!/usr/bin/env python3
"""Read completed rpicam JSON metadata objects during live capture.
Diagnostic only; does NOT pair metadata with YUV frames.
"""
import argparse
import json
import pathlib
import signal
import subprocess
import tempfile
import time

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--seconds",type=float,default=6)
    args=ap.parse_args()
    with tempfile.TemporaryDirectory(prefix="ov5647_stream_meta_") as td:
        path=pathlib.Path(td)/"meta.json"
        cmd=["rpicam-vid","--camera","0","--width","640","--height","480",
             "--framerate","60","--codec","yuv420","--nopreview",
             "--timeout","0","--output","/dev/null",
             "--metadata",str(path),"--metadata-format","json"]
        proc=subprocess.Popen(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        decoder=json.JSONDecoder()
        buffer=""
        consumed=0
        parsed=0
        first_live=None
        last_ts=None
        nonmonotonic=0
        missing=0
        polls=0
        start=time.monotonic()
        try:
            while time.monotonic()-start<args.seconds and proc.poll() is None:
                time.sleep(0.1)
                polls+=1
                if not path.exists():
                    continue
                with path.open("r",encoding="utf-8") as f:
                    f.seek(consumed)
                    chunk=f.read()
                    consumed=f.tell()
                buffer+=chunk
                while True:
                    buffer=buffer.lstrip(" \r\n\t,[]")
                    if not buffer:
                        break
                    try:
                        item,end=decoder.raw_decode(buffer)
                    except json.JSONDecodeError:
                        break
                    buffer=buffer[end:]
                    if not isinstance(item,dict):
                        continue
                    parsed+=1
                    if first_live is None:
                        first_live=time.monotonic()-start
                    ts=item.get("SensorTimestamp")
                    if not isinstance(ts,(int,float)):
                        missing+=1
                    elif last_ts is not None and ts<=last_ts:
                        nonmonotonic+=1
                    if isinstance(ts,(int,float)):
                        last_ts=ts
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        final_rows=None
        try:
            final_rows=json.loads(path.read_text(encoding="utf-8"))
        except (OSError,json.JSONDecodeError):
            pass
        total=len(final_rows) if isinstance(final_rows,list) else -1
        print(f"LIVE_PARSE parsed_during_capture={parsed} final_metadata={total} "
              f"first_live_s={first_live} polls={polls} missing_sensor_ts={missing} "
              f"nonmonotonic_sensor_ts={nonmonotonic} rc={proc.returncode}")
        print("NOTE no_frame_pairing_or_imu_sync_claimed")
        return 0 if parsed>0 and missing==0 and nonmonotonic==0 and total>=parsed else 1

if __name__=="__main__":
    raise SystemExit(main())
