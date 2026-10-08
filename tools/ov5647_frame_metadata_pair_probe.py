#!/usr/bin/env python3
"""Validate YUV420 frame count vs rpicam-vid metadata count.
Diagnostic only. Does not establish frame-to-metadata one-to-one identity.
"""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile

WIDTH=640
HEIGHT=480
FRAME_BYTES=WIDTH*HEIGHT*3//2

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--seconds",type=int,default=5)
    args=p.parse_args()
    if args.seconds<1:
        p.error("--seconds must be positive")
    with tempfile.TemporaryDirectory(prefix="ov5647_pair_") as tmp:
        video=Path(tmp)/"video.yuv"
        meta=Path(tmp)/"metadata.json"
        cmd=["rpicam-vid","--camera","0","--width",str(WIDTH),
             "--height",str(HEIGHT),"--framerate","60",
             "--codec","yuv420","--nopreview",
             "--timeout",str(args.seconds*1000),
             "--output",str(video),
             "--metadata",str(meta),"--metadata-format","json"]
        print("PAIR_PROBE running",flush=True)
        try:
            r=subprocess.run(cmd,capture_output=True,text=True,
                             timeout=args.seconds+15)
        except subprocess.TimeoutExpired:
            print("PAIR_PROBE_FAIL timeout")
            return 2
        if r.returncode:
            print("PAIR_PROBE_FAIL rpicam_rc",r.returncode)
            print(r.stderr[-2000:])
            return 2
        if not video.exists() or not meta.exists():
            print("PAIR_PROBE_FAIL missing_file")
            return 2
        nbytes=video.stat().st_size
        try:
            rows=json.loads(meta.read_text())
        except (json.JSONDecodeError,UnicodeDecodeError) as e:
            print("PAIR_PROBE_FAIL metadata_parse",e)
            return 2
        if not isinstance(rows,list):
            print("PAIR_PROBE_FAIL metadata_not_array")
            return 2
        stamps=[r.get("SensorTimestamp") for r in rows]
        valid=[isinstance(x,(int,float)) for x in stamps]
        diffs=[b-a for a,b in zip(stamps,stamps[1:])
               if isinstance(a,(int,float)) and isinstance(b,(int,float))]
        monotonic=all(x>0 for x in diffs) and len(valid)>1 and all(valid)
        frames=nbytes//FRAME_BYTES
        remainder=nbytes%FRAME_BYTES
        print(f"PAIR_PROBE frames={frames} metadata={len(rows)} "
              f"delta={frames-len(rows)} yuv_bytes={nbytes} "
              f"partial_bytes={remainder} sensor_ts_all={int(all(valid))} "
              f"sensor_ts_monotonic={int(monotonic)}")
        if diffs:
            sd=sorted(diffs)
            print(f"SENSOR_GAP_MS min={sd[0]/1e6:.3f} "
                  f"median={sd[len(sd)//2]/1e6:.3f} "
                  f"max={sd[-1]/1e6:.3f}")
        print("NOTE frame_count_match_does_not_prove_identity")
        return 0 if frames>0 and remainder==0 and monotonic else 1

if __name__=="__main__":
    raise SystemExit(main())
