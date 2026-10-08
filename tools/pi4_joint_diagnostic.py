#!/usr/bin/env python3
"""Concurrent OV5647 WORKED5 + FC IMU diagnostic, no flight output.
Two independent logs; camera exposure timestamp is unavailable.
"""
import argparse
import csv
import pathlib
import subprocess
import sys
import time

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--out",default="")
    p.add_argument("--port",default="/dev/serial0")
    p.add_argument("--baud",type=int,default=460800)
    p.add_argument("--python",default=sys.executable)
    a=p.parse_args()
    root=pathlib.Path(__file__).resolve().parent.parent
    out=pathlib.Path(a.out or f"/tmp/pi4_joint_{time.strftime('%Y%m%d_%H%M%S')}")
    out.mkdir(parents=True,exist_ok=True)
    binary=out/"ov5647_worked5_diagnostic"
    cmd=["g++","-std=c++17","-O2","-DNDEBUG","-pthread","-I",str(root/"src"),
         str(root/"tools/ov5647_worked5_diagnostic.cpp"),"-o",str(binary)]
    cmd+=subprocess.check_output(["pkg-config","--cflags","--libs","opencv4"],text=True).split()
    subprocess.run(cmd,check=True,cwd=root)
    fc=[a.python,str(root/"tools/pi4_fc_readonly_capture.py"),
        "--port",a.port,"--baud",str(a.baud),"--seconds","18",
        "--out",str(out/"fc.csv")]
    with (out/"camera.log").open("w") as camlog, (out/"fc.log").open("w") as fclog:
        fcproc=subprocess.Popen(fc,stdout=fclog,stderr=subprocess.STDOUT)
        try:
            time.sleep(1)
            camproc=subprocess.run([str(binary)],stdout=camlog,stderr=subprocess.STDOUT,timeout=25)
        finally:
            try:
                fcrc=fcproc.wait(timeout=22)
            except subprocess.TimeoutExpired:
                fcproc.terminate()
                fcrc=fcproc.wait(timeout=5)
    counts={}
    fcpath=out/"fc.csv"
    if fcpath.exists():
        with fcpath.open(newline="") as f:
            for row in csv.DictReader(f):
                k=row["msg_type"]
                counts[k]=counts.get(k,0)+1
    print(f"JOINT_DIAG dir={out} camera_rc={camproc.returncode} fc_rc={fcrc} "
          f"heartbeat={counts.get('HEARTBEAT',0)} "
          f"attitude={counts.get('ATTITUDE',0)} raw_imu={counts.get('RAW_IMU',0)} "
          f"highres_imu={counts.get('HIGHRES_IMU',0)} "
          "camera_sensor_ts=UNAVAILABLE clock_sync=UNVERIFIED no_fc_tx=1")
    print("CAMERA_LOG",out/"camera.log")
    print("FC_LOG",out/"fc.log")
    print("FC_CSV",fcpath)
    return 0 if camproc.returncode==0 and fcrc==0 else 1

if __name__=="__main__":
    raise SystemExit(main())
