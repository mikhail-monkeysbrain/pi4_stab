#!/usr/bin/env python3
"""Pi4 camera + FC telemetry + HTTP dashboard. VO TX intentionally gated."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HTML="""<!doctype html><html lang="ru"><meta charset="utf-8"><title>Pi4 WORKED5</title>
<style>body{font:18px system-ui;background:#111827;color:#f9fafb;max-width:850px;margin:40px auto}pre{background:#1f2937;padding:24px;white-space:pre-wrap}h1{font-size:28px}</style>
<h1>Pi4 WORKED5 — runtime</h1><p>VO → FC: заблокировано (нет метрической AGL)</p>
<pre id="status">Подключение...</pre><script>
setInterval(async()=>{try{let r=await fetch('/api/status');document.getElementById('status').textContent=JSON.stringify(await r.json(),null,2)}catch(e){}},500);
</script></html>"""


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--port",default="/dev/serial0")
    ap.add_argument("--baud",type=int,default=460800)
    ap.add_argument("--http-port",type=int,default=8080)
    args=ap.parse_args()
    lock_file=open('/tmp/pi4_stab_camera_runtime.lock','w')
    try:
        fcntl.flock(lock_file,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        print('PI4_CAMERA_BUSY: другой runtime использует камеру',flush=True)
        return 3
    from pymavlink import mavutil
    root=Path(__file__).resolve().parent.parent
    state={"camera":"STARTING","fc":"CONNECTING","flow_received":0,"flow_invalid":0,
           "flow_last_age_s":None,"fc_messages":0,"attitude":None,"local_position":None,
           "vo_tx":"BLOCKED","reason":"No verified metric AGL; provisional OV5647 calibration",
           "runtime":"RUNNING","armed":False,"ekf_valid":False,
           "visual_flow_u":0.0,"visual_flow_v":0.0,"visual_flow_units":"normalized image displacement",
           "visual_flow_samples":0,"visual_flow_last":None}
    lock=threading.Lock()
    stop=threading.Event()
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path=="/api/status":
                with lock:
                    payload=dict(state)
                    if payload.pop("_last_flow",None) is not None:
                        payload["flow_last_age_s"]=round(time.monotonic()-state["_last_flow"],3)
                last_position=payload.pop("_last_position",None)
                payload["local_position_age_s"]=round(time.monotonic()-last_position,3) if last_position else None
                payload["ekf_valid"]=bool(last_position and time.monotonic()-last_position<1.0)
                data=json.dumps(payload).encode()
                self.send_response(200);self.send_header("Content-Type","application/json; charset=utf-8")
            elif self.path=="/":
                data=HTML.encode()
                self.send_response(200);self.send_header("Content-Type","text/html; charset=utf-8")
            else:
                self.send_error(404);return
            self.send_header("Content-Length",str(len(data)));self.end_headers();self.wfile.write(data)
        def log_message(self,*args):pass
    server=ThreadingHTTPServer(("0.0.0.0",args.http_port),Handler)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    with tempfile.TemporaryDirectory(prefix="pi4_web_") as directory:
        path=str(Path(directory)/"flow.sock")
        sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);sock.bind(path);sock.settimeout(0.05)
        binary=Path(directory)/"adapter"
        flags=subprocess.check_output(["pkg-config","--cflags","--libs","libcamera","opencv4"],text=True).split()
        subprocess.run(["g++","-std=c++17","-O2","-pthread","-Isrc",
                        "tools/pi4_ov5647_adapter_worked5.cpp","-o",str(binary),*flags],
                       cwd=root,check=True)
        env=dict(os.environ,PI4_FLOW_SOCKET=path,PI4_PREVIEW_UDP_PORT="8766")
        config_path=root/"config/runtime.json"
        if config_path.exists():
            settings=json.loads(config_path.read_text(encoding="utf-8"))
            roi=settings.get("feature_roi",[0.2,0.32,0.8,0.9])
            env["MONKEYS_FEATURE_ROI"]=" ".join(str(x) for x in roi)
            env["MONKEYS_MAX_FEATURES"]=str(settings.get("max_features",500))
        # Adapter currently supports max 120 seconds. Restart on successful completion.
        camera=None
        fc=None
        previous=0
        print(f"PI4_WEB_RUNTIME http://0.0.0.0:{args.http_port} vo_tx=BLOCKED",flush=True)
        try:
            while not stop.is_set():
                if camera is not None and camera.poll() is not None and camera.returncode != 0:
                    with lock:state['camera']='FAILED';state['runtime']='CAMERA_ERROR'
                    print('PI4_CAMERA_ERROR: acquire failed; auto restart disabled',flush=True)
                    break
                if camera is None or camera.poll() is not None:
                    camera=subprocess.Popen([str(binary),"120"],cwd=root,env=env)
                    with lock:state["camera"]="RUNNING"
                if fc is None:
                    try:
                        fc=mavutil.mavlink_connection(args.port,baud=args.baud,autoreconnect=True)
                        with lock:state["fc"]="CONNECTED"
                        for message_id,interval_us in ((32,100000),(30,100000),(0,1000000)):
                            fc.mav.command_long_send(fc.target_system or 1,fc.target_component or 1,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,
                                message_id,interval_us,0,0,0,0,0)
                    except Exception as exc:
                        with lock:state["fc"]=f"ERROR: {exc}"
                        time.sleep(1)
                try:
                    packet=sock.recv(4096).decode("ascii").split(",")
                    ts=int(packet[0]);dt=float(packet[1])
                    if len(packet)!=8 or ts<=previous or not 0<dt<0.2 or packet[7]!="0":
                        raise ValueError("invalid flow packet")
                    previous=ts
                    with lock:
                        state["flow_received"]+=1;state["_last_flow"]=time.monotonic()
                        # Display-only integral: NOT metres and never transmitted to FC.
                        du=float(packet[3]);dv=float(packet[4])
                        if all(__import__("math").isfinite(v) and abs(v)<1.0 for v in (du,dv)):
                            state["visual_flow_u"]+=du
                            state["visual_flow_v"]+=dv
                            state["visual_flow_samples"]+=1
                            state["visual_flow_last"]={"du":du,"dv":dv,"tracked":int(packet[2])}
                except socket.timeout:pass
                except (ValueError,IndexError,UnicodeDecodeError):
                    with lock:state["flow_invalid"]+=1
                if fc is not None:
                    try:
                        msg=fc.recv_match(blocking=False)
                        if msg:
                            with lock:
                                state["fc_messages"]+=1
                                if msg.get_type()=="LOCAL_POSITION_NED":
                                    state["local_position"]={"x":msg.x,"y":msg.y,"z":msg.z,"vx":msg.vx,"vy":msg.vy,"vz":msg.vz}
                                    state["_last_position"]=time.monotonic()
                                if msg.get_type()=="HEARTBEAT":
                                    state["armed"]=bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                                if msg.get_type()=="ATTITUDE":
                                    state["attitude"]={"roll":round(msg.roll,3),"pitch":round(msg.pitch,3),"yaw":round(msg.yaw,3)}
                    except Exception as exc:
                        with lock:state["fc"]=f"ERROR: {exc}"
                        fc=None
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            if camera and camera.poll() is None:
                camera.terminate();camera.wait(timeout=5)
            sock.close();server.shutdown();server.server_close()
            with lock:state["runtime"]="STOPPED"
            print("PI4_WEB_RUNTIME_STOPPED",flush=True)


if __name__=="__main__":
    main()
