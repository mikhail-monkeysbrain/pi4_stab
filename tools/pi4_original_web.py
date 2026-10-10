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
    active=threading.Event()
    active.set()
    lifecycle=threading.Lock()
    last_logged_flow=0
    last_logged_fc=0
    web.log_event("WARN","Pi4: VO TX отключён, AGL отсутствует")
    lock=threading.Lock()
    def runtime():
        nonlocal proc,last_logged_flow,last_logged_fc
        env=dict(os.environ)
        while not stop.is_set():
            if not active.wait(0.2):
                continue
            with lifecycle:
                if proc is not None and proc.poll() is None:
                    break
                proc=subprocess.Popen([sys.executable,"-u",str(root/"tools/pi4_web_runtime.py"),
             "--port",args.serial,"--baud",str(args.baud),"--http-port","18080"],
             cwd=root,env=env, start_new_session=True)
            web.log_event("INFO","Pi4 WORKED5 runtime запущен")
            break
        while not stop.wait(0.4):
            if not active.is_set():
                continue
            try:
                import urllib.request
                with urllib.request.urlopen("http://127.0.0.1:18080/api/status",timeout=0.3) as resp:
                    data=json.load(resp)
                with lock:state.update(data)
                flow_count=int(data.get("flow_received",0))
                fc_count=int(data.get("fc_messages",0))
                if flow_count//100>last_logged_flow//100:
                    web.log_event("INFO",f"Pi4 WORKED5: {flow_count} результатов, invalid={data.get('flow_invalid',0)}")
                if fc_count//200>last_logged_fc//200:
                    web.log_event("INFO",f"Pi4 FC: {fc_count} сообщений MAVLink")
                last_logged_flow=flow_count
                last_logged_fc=fc_count
                raw={"type":"telemetry","mono_ns":time.monotonic_ns(),
                     "frame":data.get("flow_received",0),"valid":int(data.get("flow_received",0)>0),
                     "tracked":0,"inliers":0,"worked5_valid":bool(data.get("flow_received",0)),
                     "roll_deg":(data.get("attitude") or {}).get("roll",0)*180/3.141592653589793,
                     "pitch_deg":(data.get("attitude") or {}).get("pitch",0)*180/3.141592653589793,
                     "yaw_deg":(data.get("attitude") or {}).get("yaw",0)*180/3.141592653589793,
                     "range_m":None,"ekf_valid":bool(data.get("ekf_valid")),"armed":bool(data.get("armed")),
                     "x":(data.get("local_position") or {}).get("x",0),
                     "y":(data.get("local_position") or {}).get("y",0),
                     "z":(data.get("local_position") or {}).get("z",0),
                     "flow_sent":False,"range_sent":False}
                udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                udp.sendto(json.dumps(raw).encode(),("127.0.0.1",web.LIVE_UDP_PORT))
                udp.close()
            except Exception:
                pass
            if proc.poll() is not None:
                with lock:state["camera"]="STOPPED"
                break
    # Keep FC 3D in real metres. Show WORKED5 separately in unscaled image coordinates.
    visual_panel = r"""
<div id="pi4-flow-panel" style="position:fixed;right:12px;bottom:12px;z-index:30;background:#07121cef;border:1px solid #426078;border-radius:8px;padding:8px;color:#d6e9fa;font:12px sans-serif;pointer-events:none">
 <div>WORKED5 · траектория без масштаба (НЕ метры)</div>
 <canvas id="pi4-flow-canvas" width="260" height="170"></canvas>
 <div id="pi4-flow-info">Ожидание кадров...</div>
</div>
<script>
(function(){
 const c=document.getElementById('pi4-flow-canvas'),ctx=c.getContext('2d');
 let trail=[],lastSamples=0,origin=null;
 async function tick(){
  try{
   const r=await fetch('/api/pi4/status',{cache:'no-store'}),d=await r.json();
   const u=Number(d.visual_flow_u),v=Number(d.visual_flow_v),n=Number(d.visual_flow_samples);
   if(!Number.isFinite(u)||!Number.isFinite(v)||!Number.isFinite(n))return;
   if(n<lastSamples){trail=[];origin=null}
   if(!origin)origin=[u,v];
   if(n!==lastSamples){trail.push([u-origin[0],v-origin[1]]);if(trail.length>400)trail.shift();lastSamples=n}
   ctx.fillStyle='#07121c';ctx.fillRect(0,0,c.width,c.height);
   ctx.strokeStyle='#24445d';ctx.beginPath();ctx.moveTo(130,0);ctx.lineTo(130,170);ctx.moveTo(0,85);ctx.lineTo(260,85);ctx.stroke();
   const extent=Math.max(.02,...trail.map(p=>Math.max(Math.abs(p[0]),Math.abs(p[1]))));
   const scale=Math.min(110/extent,4000);
   ctx.strokeStyle='#20b8ff';ctx.lineWidth=2;ctx.beginPath();
   trail.forEach((p,i)=>{const x=130+p[0]*scale,y=85+p[1]*scale;i?ctx.lineTo(x,y):ctx.moveTo(x,y)});
   ctx.stroke();
   const last=trail[trail.length-1]||[0,0];ctx.fillStyle='#17d878';ctx.beginPath();ctx.arc(130+last[0]*scale,85+last[1]*scale,4,0,Math.PI*2);ctx.fill();
   document.getElementById('pi4-flow-info').textContent='Кадры: '+n+' · FC EKF: '+(d.ekf_valid?'есть позиция':'нет позиции');
  }catch(e){document.getElementById('pi4-flow-info').textContent='Нет связи с WORKED5'}
 }
 setInterval(tick,400);tick();
})();
</script>
"""
    web.HTML=web.HTML.replace("</body>",visual_panel+"</body>")
    original_get=web.H.do_GET
    def safe_get(self):
        if self.path=="/api/pi4/status":
            with lock:payload=dict(state)
            payload["vo_tx"]="BLOCKED"
            return self.send_json(payload)
        return original_get(self)
    def safe_post(self):
        from urllib.parse import urlparse
        path=urlparse(self.path).path
        if path=="/api/start":
            with lifecycle:
                if proc is not None and proc.poll() is None:
                    return self.send_json({"ok":True,"already_running":True,"pid":proc.pid})
                active.set()
                threading.Thread(target=runtime,daemon=True).start()
            web.log_event("INFO","Pi4: запуск runtime запрошен из вебморды")
            return self.send_json({"ok":True,"starting":True})
        if path=="/api/stop":
            active.clear()
            with lifecycle:
                if proc is not None and proc.poll() is None:
                    proc.terminate()
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:proc.kill()
            web.log_event("INFO","Pi4: runtime остановлен из вебморды")
            return self.send_json({"ok":True})
        if path=="/api/zero":
            web.set_zero()
            return self.send_json({"ok":True})
        if path=="/api/system/visualization":
            return original_post(self)
        return self.send_json({"error":"Pi4: эта команда недоступна; FC control и VO TX отключены"},403)
    original_post=web.H.do_POST
    web.H.do_GET=safe_get
    web.H.do_POST=safe_post
    web.running=lambda:active.is_set() and proc is not None and proc.poll() is None
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
        active.clear()
        server.shutdown()
        web.stop_live_udp_listener()
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill()

if __name__=="__main__":
    main()
