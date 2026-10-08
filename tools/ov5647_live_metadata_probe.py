#!/usr/bin/env python3
"""Check whether rpicam-vid flushes metadata while recording, not only at exit.
No FC commands, no WORKED5 changes. Stops camera after sampling.
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
    a=ap.parse_args()
    with tempfile.TemporaryDirectory(prefix="ov5647_live_meta_") as td:
        meta=pathlib.Path(td)/"meta.json"
        cmd=["rpicam-vid","--camera","0","--width","640","--height","480",
             "--framerate","60","--codec","yuv420","--nopreview",
             "--timeout","0","--output","/dev/null",
             "--metadata",str(meta),"--metadata-format","json"]
        proc=subprocess.Popen(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,
                              start_new_session=True)
        observations=[]
        try:
            start=time.monotonic()
            while time.monotonic()-start<a.seconds:
                time.sleep(0.5)
                if proc.poll() is not None:
                    break
                raw=meta.read_text(errors="replace") if meta.exists() else ""
                count=None
                try:
                    rows=json.loads(raw)
                    count=len(rows) if isinstance(rows,list) else None
                except json.JSONDecodeError:
                    pass
                observations.append((round(time.monotonic()-start,2),len(raw),count,
                                     raw[-100:].replace("\n"," ")))
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
            try:
                _,stderr=proc.communicate(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
                _,stderr=proc.communicate()
        final=meta.read_text(errors="replace") if meta.exists() else ""
        try:
            rows=json.loads(final)
            final_count=len(rows) if isinstance(rows,list) else -1
        except json.JSONDecodeError:
            final_count=-1
        for sec,size,count,tail in observations:
            print(f"LIVE_META t={sec:.2f}s bytes={size} parseable_rows={count} tail={tail!r}")
        print(f"LIVE_META_FINAL rc={proc.returncode} bytes={len(final)} "
              f"parseable_rows={final_count}")
        if proc.returncode not in (0,-signal.SIGINT):
            print("CAMERA_STDERR",stderr.decode(errors="replace")[-1800:])
        return 0 if final_count>0 else 1

if __name__=="__main__":
    raise SystemExit(main())
