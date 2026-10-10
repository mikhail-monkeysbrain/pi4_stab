#!/usr/bin/env python3
"""Live WORKED5 transport observer: local datagrams only, never MAVLink TX."""
import argparse
import socket
import subprocess
import tempfile
import time
from pathlib import Path


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--seconds",type=int,default=8)
    args=ap.parse_args()
    if not 5<=args.seconds<=120:
        ap.error("seconds must be 5..120")
    root=Path(__file__).resolve().parent.parent
    with tempfile.TemporaryDirectory(prefix="pi4_live_") as directory:
        path=str(Path(directory)/"flow.sock")
        sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        sock.bind(path)
        sock.settimeout(0.5)
        binary=Path(directory)/"adapter"
        flags=subprocess.check_output(["pkg-config","--cflags","--libs","libcamera","opencv4"],text=True).split()
        subprocess.run(["g++","-std=c++17","-O2","-pthread","-Isrc",
                        "tools/pi4_ov5647_adapter_worked5.cpp","-o",str(binary),*flags],
                       cwd=root,check=True)
        import os
        env=dict(os.environ,PI4_FLOW_SOCKET=path)
        proc=subprocess.Popen([str(binary),str(args.seconds)],cwd=root,env=env)
        received=0
        invalid=0
        prev=0
        started=time.monotonic()
        try:
            while proc.poll() is None or time.monotonic()-started<args.seconds+1:
                try:
                    packet=sock.recv(4096).decode("ascii")
                except socket.timeout:
                    if proc.poll() is not None:
                        break
                    continue
                try:
                    fields=packet.split(",")
                    ts=int(fields[0])
                    dt=float(fields[1])
                    assert len(fields)==8 and ts>prev and 0<dt<0.2 and fields[7]=="0"
                    prev=ts
                    received+=1
                except (ValueError,AssertionError,IndexError):
                    invalid+=1
                if time.monotonic()-started>args.seconds+10:
                    break
        finally:
            if proc.poll() is None:
                proc.terminate()
            proc.wait()
            sock.close()
        print(f"PI4_LIVE_FLOW received={received} invalid={invalid} camera_rc={proc.returncode} mavlink_tx=OFF metric_valid=0")
        return 0 if received>0 and invalid==0 and proc.returncode==0 else 1


if __name__=="__main__":
    raise SystemExit(main())
