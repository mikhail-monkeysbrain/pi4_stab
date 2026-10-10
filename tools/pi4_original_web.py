#!/usr/bin/env python3
"""Original MonkeysStab dashboard, Pi4 camera/FC backend; no flight control TX."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import ThreadingHTTPServer

sys.path.insert(0,str(Path(__file__).resolve().parent))
import web_service as web

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--port",type=int,default=8080)
    ap.add_argument("--serial",default="/dev/serial0")
    ap.add_argument("--baud",type=int,default=460800)
    args=ap.parse_args()
    root=Path(__file__).resolve().parent.parent
    web.RUN_ROOT.mkdir(parents=True,exist_ok=True)
    proc=None
    state={"flow_received":0,"fc_messages":0,"fc":"CONNECTING","camera":"STARTING"}
    stop=threading.Event()
    lock=threading.Lock()
    def runtime():
        nonlocal proc
        env=dict(os.environ)
        proc=subprocess.Popen([sys.executable,"-u",str(root/"tools/pi4_web_runtime.py"),
             "--port",args.serial,"--baud",str(args.baud),"--http-port","18080"],
             cwd=root,env=env)
        while not stop.wait(0.4):
            try:
                import urllib.request
                with urllib.request.urlopen("http://127.0.0.1:18080/api/status",timeout=0.3) as resp:
                    data=json.load(resp)
                with lock:state.update(data)
                raw={"type":"telemetry","mono_ns":time.monotonic_ns(),
                     "frame":data.get("flow_received",0),"valid":int(data.get("flow_received",0)>0),
                     "tracked":0,"inliers":0,"worked5_valid":bool(data.get("flow_received",0)),
                     "roll_deg":(data.get("attitude") or {}).get("roll",0)*180/3.141592653589793,
                     "pitch_deg":(data.get("attitude") or {}).get("pitch",0)*180/3.141592653589793,
                     "yaw_deg":(data.get("attitude") or {}).get("yaw",0)*180/3.141592653589793,
                     "range_m":None,"ekf_valid":False,"armed":False,
                     "flow_sent":False,"range_sent":False}
                udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                udp.sendto(json.dumps(raw).encode(),("127.0.0.1",web.LIVE_UDP_PORT))
                udp.close()
            except Exception:
                pass
            if proc.poll() is not None:
                with lock:state["camera"]="STOPPED"
                break
    original_get=web.H.do_GET
    def safe_get(self):
        if self.path=="/api/pi4/status":
            with lock:payload=dict(state)
            payload["vo_tx"]="BLOCKED"
            return self.send_json(payload)
        return original_get(self)
    def safe_post(self):
        self.send_json({"error":"Pi4: команды FC и старого runtime отключены; VO TX заблокирован"},403)
    web.H.do_GET=safe_get
    web.H.do_POST=safe_post
    web.running=lambda:proc is not None and proc.poll() is None
    web.start_live_udp_listener()
    threading.Thread(target=runtime,daemon=True).start()
    server=ThreadingHTTPServer(("0.0.0.0",args.port),web.H)
    print(f"PI4 ORIGINAL WEB http://0.0.0.0:{args.port}  FC_CONTROL=OFF VO_TX=OFF",flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.shutdown()
        web.stop_live_udp_listener()
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill()

if __name__=="__main__":
    main()
