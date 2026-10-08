#!/usr/bin/env python3
"""Check rpicam-vid frame metadata without modifying WORKED5 or FC.
Requires rpicam-vid support for --metadata and --metadata-format.
"""
import argparse
import json
import pathlib
import subprocess
import tempfile

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--seconds",type=int,default=5)
    args=ap.parse_args()
    with tempfile.TemporaryDirectory(prefix="ov5647_metadata_") as tmp:
        path=pathlib.Path(tmp)/"frames.json"
        cmd=["rpicam-vid","--camera","0","--width","640","--height","480",
             "--framerate","60","--codec","yuv420","--nopreview",
             "--timeout",str(args.seconds*1000),"--output","/dev/null",
             "--metadata",str(path),"--metadata-format","json"]
        print("METADATA_PROBE running",flush=True)
        try:
            proc=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                                text=True,timeout=args.seconds+12)
        except subprocess.TimeoutExpired:
            print("METADATA_FAIL timeout")
            return 2
        if proc.returncode:
            print("METADATA_FAIL rpicam_rc=",proc.returncode)
            print(proc.stderr[-3000:])
            return 2
        if not path.exists() or path.stat().st_size==0:
            print("METADATA_FAIL empty_metadata")
            return 2
        raw=path.read_text(errors="replace")
        print("METADATA_FILE bytes=",len(raw))
        try:
            data=json.loads(raw)
            if isinstance(data,dict):
                rows=[data]
            elif isinstance(data,list):
                rows=data
            else:
                rows=[]
        except json.JSONDecodeError:
            rows=[]
            for line in raw.splitlines():
                try:
                    row=json.loads(line.strip().rstrip(","))
                    if isinstance(row,dict):rows.append(row)
                except json.JSONDecodeError:
                    pass
        if not rows:
            print("METADATA_PARSE_UNKNOWN sample=",raw[:1400])
            return 1
        print("METADATA_ROWS",len(rows))
        print("METADATA_KEYS",sorted(rows[0]))
        for i,row in enumerate(rows[:3]):
            relevant={k:v for k,v in row.items() if
                      any(x in k.lower() for x in
                          ("timestamp","frame","exposure","sensor","sequence"))}
            print("METADATA_SAMPLE",i,json.dumps(relevant,ensure_ascii=False))
        print("METADATA_PROBE_PASS parseable=1")
        return 0

if __name__=="__main__":
    raise SystemExit(main())
