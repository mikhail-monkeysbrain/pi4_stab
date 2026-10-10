#!/usr/bin/env python3
import base64
import csv
import hashlib
import io
import json
import math
import re
import struct
import os
import signal
import socket
import shutil
import subprocess
import threading
import time
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from collections import deque
from pathlib import Path
from urllib.parse import urlparse, parse_qs, quote

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/"config"/"runtime.json"
GEOMETRY=ROOT/"config"/"mount_geometry.json"
FC_PROFILE=ROOT/"config"/"fc_profile.json"
RUN_ROOT=Path(os.environ.get("MONKEYS_RUN_ROOT", str(Path.home()/"monkeysStab_runs")))
WEB_LOG=RUN_ROOT/"web_runtime.log"
WEB_FORENSIC=RUN_ROOT/"web_forensic.csv"
WEB_ASSETS=ROOT/"web_assets"
PREVIEW_PATH=Path(os.environ.get("MONKEYS_WEB_PREVIEW_PATH","/dev/shm/monkeysstab_vo_preview.jpg"))
DEFAULTS={
    "focal_scale":0.931,
    "feature_roi":[0.20,0.32,0.80,0.90],
    "max_features":500,
    "local_gui":True,
    "visualization_mode":"simple",
}

_lock=threading.RLock()
_proc=None
_log_handle=None
_runtime_started_wall=0.0
_runtime_test_duration_s=0.0
_runtime_test_deadline_mono=0.0
_runtime_test_timer_token=0
_active_csv=None
_router_proc=None
_router_log_handle=None
_router_started_here=False
_statustext_proc=None
_statustext_thread=None
_fc_blackbox_proc=None
_fc_blackbox_log_handle=None
_zero={"x":None,"y":None,"z":None}
_yaw_zero_deg=None
_raw_zero={"n":None,"e":None}
_imu_zero={"n":None,"e":None,"d":None}
_fused_zero={"n":None,"e":None}
_camvc_zero={"n":None,"e":None}
_journal=deque(maxlen=500)
_messages=deque(maxlen=500)
_ws_clients=set()
_ws_lock=threading.RLock()
_live_latest=None
_live_last_wall=0.0
_live_udp_thread=None
_live_udp_stop=threading.Event()
_live_udp_rx=0
_live_udp_bad=0
# LIVE_UDP_FORENSIC_V1
_live_udp_last_datagram_wall=0.0
_live_udp_last_telemetry_wall=0.0
_recovery_state="RUNNING"
_recovery_lock=threading.Lock()
_recovery_thread=None
_recovery_fault_since=0.0
_recovery_armed=True
_recovery_last_restart_wall=0.0
_camera_jpeg=None
_camera_last_wall=0.0
_last_rc_zero_seq=None
_run_record_handle=None
_run_record_tmp=None
_run_record_started_wall=0.0
_run_record_pending=None
_run_raw_dir=None
_run_raw_pending=None
_forensic_handle=None
FORENSIC_COLUMNS=["wall_time","mono_ns","frame","kind","action","detail","fc_n_m","fc_e_m","fc_d_m","fc_vn_mps","fc_ve_mps","fc_vd_mps","web_x_mm","web_y_mm","web_z_mm","web_yaw_deg","home_n_m","home_e_m","home_d_m","home_yaw_deg","range_m","ekf_valid","armed","raw_of_n_mm","raw_of_e_mm","worked5_dN_m","worked5_dE_m","flow_sent","range_sent"]
RAW_DATASET_CONTROL=Path("/tmp/monkeysstab_raw_dataset_path")
RUN_RECORD_DIR=RUN_ROOT/"recordings"
RUN_RECORD_COLUMNS=[
    "wall_time","mono_ns","frame","valid","quality","features","tracked","inliers",
    "range_m","range_age_ms","armed","ekf_valid","x_mm","y_mm","z_mm","ekf_drift_mm",
    "raw_of_valid","raw_of_n_mm","raw_of_e_mm","raw_of_drift_mm","raw_of_vn","raw_of_ve",
    "worked5_valid","worked5_dN_m","worked5_dE_m",
    "variant_b_publish_mode","variant_b_ready","variant_b_source","variant_b_flow_x","variant_b_flow_y",
    "flow_sent","range_sent",
    "vx","vy","vz","roll_deg","pitch_deg","yaw_deg"
]
LIVE_UDP_PORT=int(os.environ.get("MONKEYS_WEB_TELEMETRY_UDP_PORT","8766"))
FC_ENDPOINT="tcp://127.0.0.1:5760"
GEOMETRY_PARAMS=["FLOW_POS_X","FLOW_POS_Y","FLOW_POS_Z","RNGFND1_POS_X","RNGFND1_POS_Y","RNGFND1_POS_Z"]

def forensic_log(kind, action="", detail="", sample=None, raw_fc=None):
    """Единая постоянная шкала Web-действий и телеметрии FC/Web."""
    global _forensic_handle
    try:
        s=dict(sample) if isinstance(sample,dict) else {}; r=dict(raw_fc) if isinstance(raw_fc,dict) else {}
        with _lock:
            if _forensic_handle is None or _forensic_handle.closed:
                RUN_ROOT.mkdir(parents=True,exist_ok=True)
                new_file=not WEB_FORENSIC.exists() or WEB_FORENSIC.stat().st_size==0
                _forensic_handle=open(WEB_FORENSIC,"a",encoding="utf-8",newline="",buffering=1)
                if new_file: csv.DictWriter(_forensic_handle,fieldnames=FORENSIC_COLUMNS).writeheader()
            row={k:"" for k in FORENSIC_COLUMNS}
            row.update({"wall_time":time.strftime("%Y-%m-%d %H:%M:%S")+"."+f"{int((time.time()%1)*1000):03d}","mono_ns":s.get("mono_ns",r.get("mono_ns","")),"frame":s.get("frame",r.get("frame","")),"kind":kind,"action":action,"detail":detail,"fc_n_m":r.get("x",s.get("_fc_n_m","")),"fc_e_m":r.get("y",s.get("_fc_e_m","")),"fc_d_m":r.get("z",s.get("_fc_d_m","")),"fc_vn_mps":r.get("vx",s.get("vx","")),"fc_ve_mps":r.get("vy",s.get("vy","")),"fc_vd_mps":r.get("vz",s.get("vz","")),"web_x_mm":s.get("x_mm",""),"web_y_mm":s.get("y_mm",""),"web_z_mm":s.get("z_mm",""),"web_yaw_deg":s.get("yaw_deg",""),"home_n_m":_zero.get("x"),"home_e_m":_zero.get("y"),"home_d_m":_zero.get("z"),"home_yaw_deg":_yaw_zero_deg,"range_m":s.get("range_m",r.get("range_m","")),"ekf_valid":s.get("ekf_valid",r.get("ekf_valid","")),"armed":s.get("armed",r.get("armed","")),"raw_of_n_mm":s.get("raw_of_n_mm",""),"raw_of_e_mm":s.get("raw_of_e_mm",""),"worked5_dN_m":s.get("worked5_dN_m",""),"worked5_dE_m":s.get("worked5_dE_m",""),"flow_sent":s.get("flow_sent",""),"range_sent":s.get("range_sent","")})
            csv.DictWriter(_forensic_handle,fieldnames=FORENSIC_COLUMNS).writerow(row); _forensic_handle.flush()
    except Exception as e:
        try: print("WEB_FORENSIC ERROR:",e,flush=True)
        except Exception: pass

def forensic_action(action, detail=""):
    with _lock: s=dict(_live_latest) if isinstance(_live_latest,dict) else {}
    forensic_log("ACTION",action,detail,sample=s)

def load_json(path, fallback):
    try:
        with open(path,"r",encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return fallback

def load_config():
    d=load_json(CONFIG, dict(DEFAULTS))
    out=dict(DEFAULTS)
    out.update({k:v for k,v in d.items() if k in out})
    return out

def validate_config(d):
    fs=float(d.get("focal_scale"))
    if not (0.5 < fs < 2.0):
        raise ValueError("focal_scale должен быть в диапазоне 0.5..2.0")
    roi=d.get("feature_roi")
    if not isinstance(roi,list) or len(roi)!=4:
        raise ValueError("feature_roi должен содержать 4 числа")
    roi=[float(x) for x in roi]
    x0,y0,x1,y1=roi
    if not (0<=x0<x1<=1 and 0<=y0<y1<=1 and x1-x0>=0.20 and y1-y0>=0.20):
        raise ValueError("feature_roi: 0..1, ширина/высота не менее 0.20")
    mf=int(d.get("max_features"))
    if not (100 <= mf <= 1000):
        raise ValueError("max_features должен быть 100..1000")
    return {
        "focal_scale":fs,
        "feature_roi":roi,
        "max_features":mf,
        "local_gui":bool(d.get("local_gui",True)),
        "visualization_mode":str(d.get("visualization_mode","simple")) if str(d.get("visualization_mode","simple")) in ("simple","light") else "simple",
    }

def save_config(d):
    merged=load_config()
    merged.update(d)
    d=validate_config(merged)
    tmp=CONFIG.with_suffix(".json.tmp")
    with open(tmp,"w",encoding="utf-8") as f:
        json.dump(d,f,ensure_ascii=False,indent=2)
        f.write("\n")
    os.replace(tmp,CONFIG)
    return d

def running():
    global _proc
    with _lock:
        return _proc is not None and _proc.poll() is None

def open_rotating_log(path, max_bytes=20*1024*1024, backups=2):
    """Open an append log after bounded size rotation."""
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    try:
        if path.exists() and path.stat().st_size >= max_bytes:
            oldest=Path(str(path)+f".{backups}")
            try: oldest.unlink()
            except FileNotFoundError: pass
            for i in range(backups-1,0,-1):
                src=Path(str(path)+f".{i}")
                if src.exists():
                    os.replace(src,Path(str(path)+f".{i+1}"))
            os.replace(path,Path(str(path)+".1"))
    except OSError:
        pass
    return open(path,"a",encoding="utf-8",buffering=1)

def runtime_exit_info():
    with _lock:
        p=_proc
    if p is None or p.poll() is None:
        return None
    tail=log_tail(45)
    return {"returncode":p.returncode,"log_tail":tail}

def _ws_frame_text(text):
    data=text.encode("utf-8")
    n=len(data)
    if n<126:
        return bytes((0x81,n))+data
    if n<65536:
        return bytes((0x81,126))+struct.pack("!H",n)+data
    return bytes((0x81,127))+struct.pack("!Q",n)+data

def _ws_frame_pong(data=b""):
    n=len(data)
    if n<126:
        return bytes((0x8A,n))+data
    return bytes((0x8A,126))+struct.pack("!H",n)+data

def ws_broadcast(obj):
    frame=_ws_frame_text(json.dumps(obj,ensure_ascii=False,separators=(",",":")))
    dead=[]
    with _ws_lock:
        clients=list(_ws_clients)
    for sock in clients:
        try:
            sock.sendall(frame)
        except Exception:
            dead.append(sock)
    if dead:
        with _ws_lock:
            for sock in dead:_ws_clients.discard(sock)

def _strip_legacy_imu_fields(payload):
    if not isinstance(payload, dict):
        return payload
    for key in list(payload):
        if key.startswith("imu_") or key.startswith("fused_"):
            payload.pop(key, None)
    return payload

def live_payload(raw):
    global _live_latest,_live_last_wall,_last_rc_zero_seq,_imu_zero,_fused_zero,_camvc_zero,_yaw_zero_deg
    try:
        x=float(raw.get("x",0.0));y=float(raw.get("y",0.0));z=float(raw.get("z",0.0))
    except Exception:
        x=y=z=0.0
    raw_n=raw.get("raw_of_n")
    raw_e=raw.get("raw_of_e")
    try:
        raw_n=float(raw_n) if raw_n is not None else None
        raw_e=float(raw_e) if raw_e is not None else None
    except Exception:
        raw_n=raw_e=None
    def _opt_float(key):
        try:
            v=raw.get(key)
            return float(v) if v is not None else None
        except Exception:
            return None
    imu_n=_opt_float("imu_dr_n_mm"); imu_e=_opt_float("imu_dr_e_mm"); imu_d=_opt_float("imu_dr_d_mm")
    fused_n=_opt_float("fused_v1_n_mm"); fused_e=_opt_float("fused_v1_e_mm")
    camvc_n=_opt_float("imu_camvc_n_mm"); camvc_e=_opt_float("imu_camvc_e_mm"); camvc_d=_opt_float("imu_camvc_d_mm")

    try:
        raw_yaw_deg=float(raw.get("yaw_deg",0.0) or 0.0)
    except Exception:
        raw_yaw_deg=0.0

    rc_zero_event=False
    try:
        rc_seq=int(raw.get("rc_zero_seq",0) or 0)
    except Exception:
        rc_seq=0

    with _lock:
        if _last_rc_zero_seq is None:
            _last_rc_zero_seq=rc_seq
        elif rc_seq!=_last_rc_zero_seq:
            _last_rc_zero_seq=rc_seq
            _zero["x"],_zero["y"],_zero["z"]=x,y,z
            _yaw_zero_deg=raw_yaw_deg
            if raw_n is not None and raw_e is not None:
                _raw_zero["n"],_raw_zero["e"]=raw_n,raw_e
            if imu_n is not None: _imu_zero["n"]=imu_n
            if imu_e is not None: _imu_zero["e"]=imu_e
            if imu_d is not None: _imu_zero["d"]=imu_d
            if fused_n is not None: _fused_zero["n"]=fused_n
            if fused_e is not None: _fused_zero["e"]=fused_e
            if camvc_n is not None: _camvc_zero["n"]=camvc_n
            if camvc_e is not None: _camvc_zero["e"]=camvc_e
            rc_zero_event=True
            log_event("INFO",f"HOME/0 с пульта: RC6={raw.get('rc6_us',0)} RC8={raw.get('rc8_us',0)} RC10={raw.get('rc10_us',0)} seq={rc_seq}")

        # Set the display HOME only from the first valid FC EKF sample.
        # Runtime startup can emit telemetry before LOCAL_POSITION_NED is valid;
        # using its default x/y/z=0 would make the later absolute NED Z
        # (e.g. -34 m) appear as a huge relative jump in the Web UI.
        if _zero["x"] is None and bool(raw.get("ekf_valid",False)):
            _zero["x"],_zero["y"],_zero["z"]=x,y,z
            _yaw_zero_deg=raw_yaw_deg
            # WEB_HOME_CAPTURE_FORENSIC_V1
            # Diagnostic only: record the exact FC sample selected as Web HOME.
            log_event(
                "INFO",
                "WEB HOME capture: "
                f"frame={int(raw.get('frame',0) or 0)} "
                f"x={x:.6f} y={y:.6f} z={z:.6f} "
                f"range_m={raw.get('range_m')} "
                f"ekf_valid={bool(raw.get('ekf_valid',False))}"
            )
        zx,zy,zz=_zero["x"],_zero["y"],_zero["z"]
        if raw_n is not None and raw_e is not None and _raw_zero["n"] is None:
            _raw_zero["n"],_raw_zero["e"]=raw_n,raw_e
        if imu_n is not None and _imu_zero["n"] is None: _imu_zero["n"]=imu_n
        if imu_e is not None and _imu_zero["e"] is None: _imu_zero["e"]=imu_e
        if imu_d is not None and _imu_zero["d"] is None: _imu_zero["d"]=imu_d
        if fused_n is not None and _fused_zero["n"] is None: _fused_zero["n"]=fused_n
        if fused_e is not None and _fused_zero["e"] is None: _fused_zero["e"]=fused_e
        if camvc_n is not None and _camvc_zero["n"] is None: _camvc_zero["n"]=camvc_n
        if camvc_e is not None and _camvc_zero["e"] is None: _camvc_zero["e"]=camvc_e
        czn,cze=_camvc_zero["n"],_camvc_zero["e"]
        rzn,rze=_raw_zero["n"],_raw_zero["e"]
        izn,ize,izd=_imu_zero["n"],_imu_zero["e"],_imu_zero["d"]
        fzn,fze=_fused_zero["n"],_fused_zero["e"]
    raw_rel_n=(raw_n-rzn) if raw_n is not None and rzn is not None else None
    raw_rel_e=(raw_e-rze) if raw_e is not None and rze is not None else None
    imu_rel_n=(imu_n-izn) if imu_n is not None and izn is not None else None
    imu_rel_e=(imu_e-ize) if imu_e is not None and ize is not None else None
    imu_rel_d=(imu_d-izd) if imu_d is not None and izd is not None else None
    fused_rel_n=(fused_n-fzn) if fused_n is not None and fzn is not None else None
    fused_rel_e=(fused_e-fze) if fused_e is not None and fze is not None else None
    camvc_rel_n=(camvc_n-czn) if camvc_n is not None and czn is not None else None
    camvc_rel_e=(camvc_e-cze) if camvc_e is not None and cze is not None else None
    ekf_has_zero=zx is not None and zy is not None and zz is not None
    # LOCAL HOME frame: +X is the vehicle nose at HOME, +Y is vehicle-right.
    # FC LOCAL_POSITION_NED is N/E/D, so position must be rotated by the same
    # HOME heading used to zero the displayed yaw.  Merely subtracting N/E
    # origins leaves the trajectory in the global NED frame.
    ned_dn=(x-zx) if ekf_has_zero else 0.0
    ned_de=(y-zy) if ekf_has_zero else 0.0
    ekf_rel_z=(z-zz) if ekf_has_zero else 0.0
    home_yaw_rad=math.radians(_yaw_zero_deg) if _yaw_zero_deg is not None else 0.0
    ekf_rel_x= math.cos(home_yaw_rad)*ned_dn + math.sin(home_yaw_rad)*ned_de
    ekf_rel_y=-math.sin(home_yaw_rad)*ned_dn + math.cos(home_yaw_rad)*ned_de
    yaw_rel_deg=((raw_yaw_deg-_yaw_zero_deg+180.0)%360.0-180.0) if _yaw_zero_deg is not None else 0.0
    out={
        "type":"telemetry",
        "available":True,
        "running":running(),
        "mono_ns":int(raw.get("mono_ns",0) or 0),
        "frame":int(raw.get("frame",0) or 0),
        "valid":int(raw.get("valid",0) or 0),
        "quality":int(raw.get("quality",0) or 0),
        "features":int(raw.get("features",0) or 0),
        "tracked":int(raw.get("tracked",0) or 0),
        "inliers":int(raw.get("inliers",0) or 0),
        "range_m":raw.get("range_m"),
        "range_age_ms":raw.get("range_age_ms"),
        "armed":bool(raw.get("armed",False)),
        "ekf_valid":bool(raw.get("ekf_valid",False)),
        "x_mm":ekf_rel_x*1000.0,
        "y_mm":ekf_rel_y*1000.0,
        "z_mm":ekf_rel_z*1000.0,
        "ekf_drift_mm":math.hypot(ekf_rel_x,ekf_rel_y)*1000.0,
        "raw_of_valid":bool(raw.get("raw_of_valid",False)),
        "raw_of_n_mm":raw_rel_n*1000.0 if raw_rel_n is not None else None,
        "raw_of_e_mm":raw_rel_e*1000.0 if raw_rel_e is not None else None,
        "raw_of_drift_mm":math.hypot(raw_rel_n,raw_rel_e)*1000.0 if raw_rel_n is not None and raw_rel_e is not None else None,
        "raw_of_vn":raw.get("raw_of_vn"),
        "raw_of_ve":raw.get("raw_of_ve"),
        "worked5_valid":bool(raw.get("worked5_valid",False)),
        "worked5_dN_m":raw.get("worked5_dN_m"),
        "worked5_dE_m":raw.get("worked5_dE_m"),
        "variant_b_publish_mode":bool(raw.get("variant_b_publish_mode",False)),
        "variant_b_ready":bool(raw.get("variant_b_ready",False)),
        "variant_b_source":raw.get("variant_b_source",0),
        "variant_b_flow_x":raw.get("variant_b_flow_x"),
        "variant_b_flow_y":raw.get("variant_b_flow_y"),
        "flow_sent":bool(raw.get("flow_sent",False)),
        "range_sent":bool(raw.get("range_sent",False)),
        "imu_dr_n_mm":imu_rel_n,
        "imu_dr_e_mm":imu_rel_e,
        "imu_dr_d_mm":imu_rel_d,
        "imu_dr_calibrated":bool(raw.get("imu_dr_calibrated",False)),
        "imu_dr_calibrating":bool(raw.get("imu_dr_calibrating",False)),
        "imu_dr_bias_samples":raw.get("imu_dr_bias_samples",0),
        "imu_dr_bias_bx":raw.get("imu_dr_bias_bx"),
        "imu_dr_bias_by":raw.get("imu_dr_bias_by"),
        "imu_dr_bias_bz":raw.get("imu_dr_bias_bz"),
        "imu_startup_res_mean_x":raw.get("imu_startup_res_mean_x"),
        "imu_startup_res_mean_y":raw.get("imu_startup_res_mean_y"),
        "imu_startup_res_mean_z":raw.get("imu_startup_res_mean_z"),
        "imu_startup_res_sd_x":raw.get("imu_startup_res_sd_x"),
        "imu_startup_res_sd_y":raw.get("imu_startup_res_sd_y"),
        "imu_startup_res_sd_z":raw.get("imu_startup_res_sd_z"),
        "imu_startup_res_norm_max":raw.get("imu_startup_res_norm_max"),
        "imu_startup_gmag_mean":raw.get("imu_startup_gmag_mean"),
        "imu_startup_gmag_sd":raw.get("imu_startup_gmag_sd"),
        "imu_startup_gmag_max":raw.get("imu_startup_gmag_max"),
        "imu_dr_vn":raw.get("imu_dr_vn"),
        "imu_dr_ve":raw.get("imu_dr_ve"),
        "imu_dr_vd":raw.get("imu_dr_vd"),
        "imu_dr_stationary":bool(raw.get("imu_dr_stationary",False)),
        "imu_dr_acc_n":raw.get("imu_dr_acc_n"),
        "imu_dr_acc_e":raw.get("imu_dr_acc_e"),
        "imu_dr_acc_d":raw.get("imu_dr_acc_d"),
        "imu_dr_amag":raw.get("imu_dr_amag"),
        "imu_dr_gmag":raw.get("imu_dr_gmag"),
        "imu_dr_acc_ok":bool(raw.get("imu_dr_acc_ok",False)),
        "imu_dr_gyro_ok":bool(raw.get("imu_dr_gyro_ok",False)),
        "imu_dr_acc_rejects":raw.get("imu_dr_acc_rejects",0),
        "imu_dr_gyro_rejects":raw.get("imu_dr_gyro_rejects",0),
        "imu_dr_stationary_samples":raw.get("imu_dr_stationary_samples",0),
        "imu_dr_dt":raw.get("imu_dr_dt"),
        "imu_cam_vn":raw.get("imu_cam_vn"),
        "imu_cam_ve":raw.get("imu_cam_ve"),
        "imu_cam_speed":raw.get("imu_cam_speed"),
        "imu_cam_age_ms":raw.get("imu_cam_age_ms"),
        "imu_cam_fresh":bool(raw.get("imu_cam_fresh",False)),
        "imu_cam_stationary":bool(raw.get("imu_cam_stationary",False)),
        "imu_zupt_shadow":bool(raw.get("imu_zupt_shadow",False)),
        "imu_zupt_shadow_accepts":raw.get("imu_zupt_shadow_accepts",0),
        "imu_zupt_shadow_blocks":raw.get("imu_zupt_shadow_blocks",0),
        "imu_cam_seq":raw.get("imu_cam_seq",0),
        "imu_camvc_active":bool(raw.get("imu_camvc_active",False)),
        "imu_camvc_stop_samples":raw.get("imu_camvc_stop_samples",0),
        "imu_camvc_activations":raw.get("imu_camvc_activations",0),
        "imu_camvc_n_mm":camvc_rel_n,
        "imu_camvc_e_mm":camvc_rel_e,
        # Camera gate constrains horizontal velocity only; vertical DR remains
        # unconstrained and must not be presented as a camera-ZUPT estimate.
        "imu_camvc_d_mm":None,
        "imu_camvc_vn":raw.get("imu_camvc_vn"),
        "imu_camvc_ve":raw.get("imu_camvc_ve"),
        "imu_camvc_vd":raw.get("imu_camvc_vd"),
        "fused_v1_visual_updates":raw.get("fused_v1_visual_updates",0),
        "fused_v1_imu_predictions":raw.get("fused_v1_imu_predictions",0),
        "fused_v1_stop_constraints":raw.get("fused_v1_stop_constraints",0),
        "fused_v1_stationary":bool(raw.get("fused_v1_stationary",False)),
        "fused_v1_stop_confirm":raw.get("fused_v1_stop_confirm",0),
        "fused_v1_n_mm":fused_rel_n,
        "fused_v1_e_mm":fused_rel_e,
        "fused_v1_vn":raw.get("fused_v1_vn"),
        "fused_v1_ve":raw.get("fused_v1_ve"),
        "rc_zero_event":rc_zero_event,
        "rc_zero_seq":rc_seq,
        "rc6_us":raw.get("rc6_us",0),
        "rc8_us":raw.get("rc8_us",0),
        "rc10_us":raw.get("rc10_us",0),
        "vx":raw.get("vx",0.0),"vy":raw.get("vy",0.0),"vz":raw.get("vz",0.0),
        "roll_deg":raw.get("roll_deg",0.0),
        "pitch_deg":raw.get("pitch_deg",0.0),
        "yaw_deg":yaw_rel_deg,
    }
    out["_fc_n_m"]=x; out["_fc_e_m"]=y; out["_fc_d_m"]=z
    forensic_log("TELEMETRY",sample=out,raw_fc=raw)
    _strip_legacy_imu_fields(out)
    with _lock:
        _live_latest=out
        _live_last_wall=time.time()
    _record_sample(out)
    return out

def start_recovery_watchdog():
    """Мониторинг свежести RAW runtime без автоматического перезапуска."""
    global _recovery_thread,_recovery_state,_recovery_fault_since
    if _recovery_thread and _recovery_thread.is_alive():
        return
    def run():
        global _recovery_state,_recovery_fault_since,_recovery_armed,_recovery_last_restart_wall
        while True:
            time.sleep(0.10)
            if not running():
                _recovery_fault_since=0.0
                continue
            # Автоматика относится только к экспериментальному RAW-контракту.
            if str(os.environ.get("MONKEYS_RAW_UNIFIED_PUBLISH","0")).lower() not in ("1","true","yes"):
                continue
            now=time.time()
            with _lock:
                sample=dict(_live_latest) if isinstance(_live_latest,dict) else None
                age=(now-_live_last_wall) if _live_last_wall else 999.0
            # Process recovery watches transport freshness.  EKF validity
            # is estimator state and must not restart an otherwise live runtime.
            healthy=(sample is not None and age < 0.35)
            if healthy:
                if _recovery_state in ("ACQUIRING","NOT_READY"):
                    _recovery_state="READY"
                elif _recovery_state not in ("RESTARTING",):
                    _recovery_state="RUNNING"
                _recovery_fault_since=0.0
                # Rearm only after a healthy interval following a restart.
                if now-_recovery_last_restart_wall > 2.0:
                    _recovery_armed=True
                continue
            if now-_runtime_started_wall < 3.5:
                continue

            # WEB_WATCHDOG_MONITOR_ONLY_V1
            # Web telemetry is diagnostic and must never restart the
            # flight-critical Optical Flow runtime.
            #
            # Before the first telemetry sample there is nothing to recover:
            # stay in ACQUIRING instead of generating the old age=999 fault.
            if sample is None:
                _recovery_state="ACQUIRING"
                _recovery_fault_since=0.0
                continue

            if _recovery_fault_since == 0.0:
                _recovery_fault_since=now
                continue

            if now-_recovery_fault_since < 1.0:
                continue

            # Log one event for this outage, but DO NOT stop/restart runtime.
            if _recovery_state != "FAULT":
                _recovery_state="FAULT"
                datagram_age=(now-_live_udp_last_datagram_wall) if _live_udp_last_datagram_wall else 999.0
                telemetry_rx_age=(now-_live_udp_last_telemetry_wall) if _live_udp_last_telemetry_wall else 999.0
                log_runtime_forensic(
                    "RECOVERY_FAULT_MONITOR_ONLY",
                    detail=(
                        f"processed_age_s={age:.3f} "
                        f"udp_datagram_age_s={datagram_age:.3f} "
                        f"udp_telemetry_age_s={telemetry_rx_age:.3f} "
                        f"udp_rx={_live_udp_rx} "
                        f"udp_bad={_live_udp_bad}"
                    )
                )
    _recovery_thread=threading.Thread(target=run,name="recovery-watchdog",daemon=True)
    _recovery_thread.start()

def start_live_udp_listener():
    global _live_udp_thread,_live_udp_rx,_live_udp_bad
    if _live_udp_thread and _live_udp_thread.is_alive():
        return
    _live_udp_stop.clear()
    def run():
        global _live_udp_rx,_live_udp_bad,_camera_jpeg,_camera_last_wall
        global _live_udp_last_datagram_wall,_live_udp_last_telemetry_wall
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try:
            sock.bind(("127.0.0.1",LIVE_UDP_PORT))
        except OSError as e:
            _live_udp_bad+=1
            log_event("ERROR",f"Live telemetry UDP bind failed 127.0.0.1:{LIVE_UDP_PORT}: {e}")
            sock.close()
            return
        sock.settimeout(0.5)
        log_event("INFO",f"Live telemetry UDP listener: 127.0.0.1:{LIVE_UDP_PORT}")
        try:
            while not _live_udp_stop.is_set():
                try:
                    data,_=sock.recvfrom(65535)
                    _live_udp_last_datagram_wall=time.time()
                except socket.timeout:
                    continue
                except OSError:
                    break
                try:
                    if data.startswith(b"MJPG"):
                        jpg=data[4:]
                        if jpg:
                            with _lock:
                                _camera_jpeg=bytes(jpg)
                                _camera_last_wall=time.time()
                        continue
                    raw=json.loads(data.decode("utf-8"))
                    if raw.get("type")!="telemetry":continue
                    _live_udp_last_telemetry_wall=time.time()
                    _live_udp_rx+=1
                    ws_broadcast(live_payload(raw))
                except Exception as e:
                    _live_udp_bad+=1
                    if _live_udp_bad<=5 or _live_udp_bad%100==0:
                        log_event("WARN","Live telemetry packet error: "+str(e))
        finally:
            sock.close()
    _live_udp_thread=threading.Thread(target=run,daemon=True,name="web-live-udp")
    _live_udp_thread.start()

def stop_live_udp_listener():
    _live_udp_stop.set()

def websocket_session(handler):
    key=handler.headers.get("Sec-WebSocket-Key","")
    if not key:
        handler.send_error(400,"Missing Sec-WebSocket-Key")
        return
    accept=base64.b64encode(hashlib.sha1(
        (key+"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")
    ).digest()).decode("ascii")
    handler.send_response(101,"Switching Protocols")
    handler.send_header("Upgrade","websocket")
    handler.send_header("Connection","Upgrade")
    handler.send_header("Sec-WebSocket-Accept",accept)
    handler.end_headers()
    sock=handler.connection
    sock.settimeout(1.0)
    with _ws_lock:_ws_clients.add(sock)
    # Send current in-memory sample immediately to a newly connected browser.
    with _lock: first=dict(_live_latest) if _live_latest else None
    if first:
        try:sock.sendall(_ws_frame_text(json.dumps(first,ensure_ascii=False,separators=(",",":"))))
        except Exception:pass
    try:
        while True:
            try:
                h=sock.recv(2)
            except socket.timeout:
                continue
            if not h or len(h)<2:break
            opcode=h[0]&0x0F
            masked=bool(h[1]&0x80)
            n=h[1]&0x7F
            if n==126:
                b=sock.recv(2)
                if len(b)!=2:break
                n=struct.unpack("!H",b)[0]
            elif n==127:
                b=sock.recv(8)
                if len(b)!=8:break
                n=struct.unpack("!Q",b)[0]
            mask=sock.recv(4) if masked else b""
            payload=b""
            while len(payload)<n:
                q=sock.recv(min(4096,n-len(payload)))
                if not q:break
                payload+=q
            if masked and len(mask)==4:
                payload=bytes(v^mask[i%4] for i,v in enumerate(payload))
            if opcode==0x8:break
            if opcode==0x9:
                try:sock.sendall(_ws_frame_pong(payload))
                except Exception:break
    except Exception:
        pass
    finally:
        with _ws_lock:_ws_clients.discard(sock)

def _record_sample(sample):
    global _run_record_handle
    with _lock:
        h=_run_record_handle
        if h is None:
            return
        row={"wall_time":time.time()}
        row.update(sample)
        try:
            csv.writer(h).writerow([row.get(k,"") for k in RUN_RECORD_COLUMNS])
            h.flush()
        except Exception as e:
            log_event("ERROR","Ошибка записи прогона: "+str(e))

def run_record_status():
    with _lock:
        return {
            "recording":_run_record_handle is not None,
            "started_wall":_run_record_started_wall or None,
            "pending_name":_run_record_pending is not None,
        }

def start_run_record():
    global _run_record_handle,_run_record_tmp,_run_record_started_wall,_run_record_pending,_run_raw_dir,_run_raw_pending
    with _lock:
        if _run_record_handle is not None:
            return {"ok":True,**run_record_status()}
        if _run_record_pending is not None:
            raise RuntimeError("Сначала сохраните имя предыдущего прогона")
        RUN_RECORD_DIR.mkdir(parents=True,exist_ok=True)
        stamp=time.strftime("%Y%m%d_%H%M%S")
        _run_record_tmp=RUN_RECORD_DIR/(".active_"+stamp+".csv")
        _run_raw_dir=RUN_RECORD_DIR/(".active_"+stamp+"_dataset")
        _run_raw_dir.mkdir(parents=True,exist_ok=False)
        try:
            commit=subprocess.run(["git","rev-parse","HEAD"],cwd=str(ROOT),text=True,capture_output=True,timeout=2).stdout.strip()
        except Exception:
            commit=""
        meta={
            "format":"monkeysStab-forensic-dataset-v1",
            "created_wall_ns":time.time_ns(),
            "git_commit":commit,
            "production_csv":str(latest_run_csv() or ""),
            "camera_calibration":"camera_calibration.yaml",
            "mount_geometry":"mount_geometry.json",
            "capture_gate":"web_named_run",
        }
        (_run_raw_dir/"session.json").write_text(json.dumps(meta,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
        shutil.copy2(ROOT/"config"/"ov9281_current_mount.yaml",_run_raw_dir/"camera_calibration.yaml")
        shutil.copy2(GEOMETRY,_run_raw_dir/"mount_geometry.json")
        ctl_tmp=RAW_DATASET_CONTROL.with_suffix(".tmp")
        ctl_tmp.write_text(str(_run_raw_dir)+"\n",encoding="utf-8")
        os.replace(ctl_tmp,RAW_DATASET_CONTROL)
        _run_raw_pending=None
        _run_record_handle=open(_run_record_tmp,"w",encoding="utf-8",newline="",buffering=1)
        csv.writer(_run_record_handle).writerow(RUN_RECORD_COLUMNS)
        _run_record_started_wall=time.time()
    log_event("INFO","Запись прогона + RAW dataset начата")
    return {"ok":True,**run_record_status()}

def stop_run_record():
    global _run_record_handle,_run_record_pending,_run_raw_dir,_run_raw_pending
    with _lock:
        if _run_record_handle is None:
            return {"ok":True,**run_record_status()}
        try:
            RAW_DATASET_CONTROL.unlink()
        except FileNotFoundError:
            pass
        try:_run_record_handle.close()
        finally:_run_record_handle=None
        _run_record_pending=_run_record_tmp
        _run_raw_pending=_run_raw_dir
        _run_raw_dir=None
    log_event("INFO","Запись прогона + RAW dataset остановлена — ожидается имя")
    return {"ok":True,**run_record_status()}

def _safe_run_name(name):
    name=str(name or "").strip()
    if not name:
        raise ValueError("Введите название прогона")
    name=re.sub(r'[\\/:*?"<>|]+',"_",name)
    name=re.sub(r"\s+"," ",name).strip(" .")
    if not name:
        raise ValueError("Некорректное название прогона")
    return name[:100]

def finalize_run_record(name):
    global _run_record_pending,_run_record_tmp,_run_record_started_wall,_run_raw_pending
    with _lock:
        src=_run_record_pending
        if src is None or not Path(src).exists():
            raise RuntimeError("Нет остановленного прогона для сохранения")
        clean=_safe_run_name(name)
        stamp=time.strftime("%Y%m%d_%H%M%S",time.localtime(_run_record_started_wall or time.time()))
        base=stamp+"_"+clean
        dst=RUN_RECORD_DIR/(base+".csv")
        raw_dst=RUN_RECORD_DIR/(base+"_dataset")
        n=2
        while dst.exists() or raw_dst.exists():
            base=stamp+"_"+clean+f"_{n}"; n+=1
            dst=RUN_RECORD_DIR/(base+".csv")
            raw_dst=RUN_RECORD_DIR/(base+"_dataset")
        os.replace(src,dst)
        if _run_raw_pending is not None and Path(_run_raw_pending).exists():
            os.replace(_run_raw_pending,raw_dst)
        _run_raw_pending=None
        _run_record_pending=None
        _run_record_tmp=None
        _run_record_started_wall=0.0
    log_event("INFO","Прогон сохранён: "+dst.name+" + RAW dataset")
    return {"ok":True,"name":dst.name,"dataset":raw_dst.name if raw_dst.exists() else None}

def list_run_records():
    RUN_RECORD_DIR.mkdir(parents=True,exist_ok=True)
    out=[]
    for p in RUN_RECORD_DIR.glob("*.csv"):
        if p.name.startswith(".active_"):continue
        try:
            st=p.stat()
            out.append({"name":p.name,"size":st.st_size,"mtime":st.st_mtime})
        except OSError:pass
    out.sort(key=lambda x:x["mtime"],reverse=True)
    return out

def _selected_run_files(names):
    available={x["name"]:RUN_RECORD_DIR/x["name"] for x in list_run_records()}
    selected=[]
    for name in names:
        if name in available:selected.append(available[name])
    if not selected:raise ValueError("Не выбраны логи")
    return selected

def log_event(level,text):
    with _lock:
        _journal.append({
            "ts":time.strftime("%Y-%m-%d %H:%M:%S"),
            "level":str(level).upper(),
            "text":str(text),
        })

def fc_param_cli(*args,timeout=35):
    ensure_router()
    env=os.environ.copy()
    env["MONKEYS_FC"]=FC_ENDPOINT
    cp=subprocess.run(
        ["bash",str(ROOT/"scripts"/"fc_params_cli.sh"),*map(str,args)],
        cwd=str(ROOT),env=env,text=True,capture_output=True,timeout=timeout
    )
    if cp.returncode!=0:
        raise RuntimeError((cp.stderr or cp.stdout or "ошибка PARAM").strip())
    return cp.stdout

def parse_param_values(text):
    out={}
    for line in text.splitlines():
        if "=" not in line or line.startswith("TARGET "): continue
        k,v=line.split("=",1)
        try: out[k.strip()]=float(v.strip().split()[0])
        except Exception: pass
    return out

def read_fc_params(names):
    if not names:return {}
    return parse_param_values(fc_param_cli("read",*names))

def profile_param_names():
    p=load_json(FC_PROFILE,{})
    return list((p.get("params") or {}).keys())

def set_profile_params(values):
    if running(): raise RuntimeError("Остановите flight runtime перед изменением параметров FC")
    st=fc_control("status")
    if st.get("armed"): raise RuntimeError("FC должен быть DISARMED")
    allowed=set(profile_param_names())
    clean={}
    for k,v in values.items():
        if k not in allowed: raise ValueError("Параметр не разрешён: "+str(k))
        fv=float(v)
        if not math.isfinite(fv): raise ValueError("Некорректное значение "+k)
        clean[k]=fv
    if not clean: raise ValueError("Нет параметров для записи")
    geom_before=read_fc_params(GEOMETRY_PARAMS)
    args=["set"]
    for k,v in clean.items(): args += [k,f"{v:.6f}"]
    fc_param_cli(*args,timeout=50)
    got=read_fc_params(list(clean.keys()))
    bad=[k for k,v in clean.items() if k not in got or not math.isclose(got[k],v,rel_tol=0,abs_tol=max(1e-6,abs(v)*1e-5))]
    if bad: raise RuntimeError("Не подтверждены: "+", ".join(bad))
    geom_after=read_fc_params(GEOMETRY_PARAMS)
    changed=[k for k in GEOMETRY_PARAMS if k in geom_before and k in geom_after and not math.isclose(geom_before[k],geom_after[k],rel_tol=0,abs_tol=1e-6)]
    if changed: raise RuntimeError("ОШИБКА БЕЗОПАСНОСТИ: изменилась геометрия: "+", ".join(changed))
    prof=load_json(FC_PROFILE,{})
    prof.setdefault("params",{}).update(got)
    tmp=FC_PROFILE.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(prof,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    os.replace(tmp,FC_PROFILE)
    log_event("INFO","Параметры FC записаны: "+", ".join(clean.keys()))
    return got

def save_local_geometry(vals):
    cfg=load_json(GEOMETRY,{"frame":"FRD","units":"m","reference":"FC_IMU","camera":{"name":"OV9281"},"rangefinder":{"name":"TF-Luna"}})
    cfg.setdefault("camera",{})["name"]="OV9281";cfg.setdefault("rangefinder",{})["name"]="TF-Luna"
    cfg["camera"].update({"x":vals["FLOW_POS_X"],"y":vals["FLOW_POS_Y"],"z":vals["FLOW_POS_Z"]})
    cfg["rangefinder"].update({"x":vals["RNGFND1_POS_X"],"y":vals["RNGFND1_POS_Y"],"z":vals["RNGFND1_POS_Z"]})
    tmp=GEOMETRY.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+"\n",encoding="utf-8");os.replace(tmp,GEOMETRY)
    yp=ROOT/"config"/"ov9281_current_mount.yaml"
    text=yp.read_text(encoding="utf-8")
    x,y,z=vals["FLOW_POS_X"],vals["FLOW_POS_Y"],vals["FLOW_POS_Z"]
    block=("data: [ 0.000000000, -1.000000000,  0.000000000,  {x:.6f},\n"
           "       -1.000000000,  0.000000000,  0.000000000,  {yflu:.6f},\n"
           "        0.000000000,  0.000000000, -1.000000000,  {zflu:.6f},\n"
           "        0.000000000,  0.000000000,  0.000000000,  1.0000 ]").format(x=x,yflu=-y,zflu=-z)
    text2,n=re.subn(r"data:\s*\[.*?1\.0000\s*\]",block,text,count=1,flags=re.S)
    if n!=1: raise RuntimeError("Не удалось обновить T_BS.data в camera YAML")
    yp.write_text(text2,encoding="utf-8")

def set_geometry(values):
    if running(): raise RuntimeError("Остановите flight runtime перед изменением геометрии")
    st=fc_control("status")
    if st.get("armed"): raise RuntimeError("FC должен быть DISARMED")
    clean={}
    for k in GEOMETRY_PARAMS:
        if k not in values: raise ValueError("Не заполнено "+k)
        v=float(values[k])
        if not math.isfinite(v) or abs(v)>2.0: raise ValueError(k+": допустимо ±2 м")
        clean[k]=v
    args=["set"]
    for k in GEOMETRY_PARAMS: args += [k,f"{clean[k]:.6f}"]
    fc_param_cli(*args,timeout=50)
    got=read_fc_params(GEOMETRY_PARAMS)
    bad=[k for k in GEOMETRY_PARAMS if k not in got or not math.isclose(got[k],clean[k],rel_tol=0,abs_tol=max(1e-6,abs(clean[k])*1e-5))]
    if bad: raise RuntimeError("Геометрия не подтверждена: "+", ".join(bad))
    save_local_geometry(got)
    log_event("INFO","Геометрия датчиков записана и синхронизирована")
    return got

def start_statustext_monitor():
    global _statustext_proc,_statustext_thread
    if _statustext_proc is not None and _statustext_proc.poll() is None:return
    ensure_router()
    env=os.environ.copy();env["MONKEYS_FC"]=FC_ENDPOINT
    _statustext_proc=subprocess.Popen(
        ["bash",str(ROOT/"scripts"/"fc_statustext_monitor.sh")],
        cwd=str(ROOT),env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,
        bufsize=1
    )
    def reader():
        if not _statustext_proc.stdout:return
        for line in _statustext_proc.stdout:
            try:
                j=json.loads(line)
                txt=str(j.get("text","")).strip()
                if not txt:continue
                with _lock:_messages.append({
                    "ts":time.strftime("%H:%M:%S"),
                    "severity":int(j.get("severity",6)),
                    "text":txt,
                })
            except Exception:
                pass
    _statustext_thread=threading.Thread(target=reader,daemon=True)
    _statustext_thread.start()

def tcp_ready(host="127.0.0.1",port=5760):
    try:
        with socket.create_connection((host,port),timeout=0.35):
            return True
    except OSError:
        return False

def ensure_router():
    global _router_proc,_router_log_handle,_router_started_here
    with _lock:
        if tcp_ready():
            return
        RUN_ROOT.mkdir(parents=True,exist_ok=True)
        router_log=RUN_ROOT/"mavlink_router_web.log"
        _router_log_handle=open_rotating_log(router_log)
        _router_log_handle.write("\n===== WEB ROUTER START %s =====\n"%time.strftime("%Y-%m-%d %H:%M:%S"))
        env=os.environ.copy()
        env["MONKEYS_FC_TCP_PORT"]="5760"
        _router_proc=subprocess.Popen(
            ["bash",str(ROOT/"scripts"/"run_mavlink_wifi.sh")],
            cwd=str(ROOT),env=env,
            stdout=_router_log_handle,stderr=subprocess.STDOUT,
            start_new_session=True,text=True
        )
        _router_started_here=True
        log_event("INFO","MAVLink router запущен")
    deadline=time.time()+6.0
    while time.time()<deadline:
        if tcp_ready(): return
        if _router_proc.poll() is not None:
            raise RuntimeError("MAVLink router завершился при запуске")
        time.sleep(0.1)
    raise RuntimeError("MAVLink router не открыл tcp://127.0.0.1:5760")

def stop_router():
    global _router_proc,_router_log_handle,_router_started_here
    with _lock:
        if not _router_started_here or _router_proc is None:
            return
        p=_router_proc
        _router_proc=None
        _router_started_here=False
    if p.poll() is None:
        try: os.killpg(p.pid,signal.SIGTERM)
        except ProcessLookupError: pass
        try: p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            try: os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError: pass
    if _router_log_handle:
        try:_router_log_handle.close()
        except Exception:pass
        _router_log_handle=None

def fc_control(*args):
    ensure_router()
    env=os.environ.copy()
    env["MONKEYS_FC"]=FC_ENDPOINT
    cp=subprocess.run(
        ["bash",str(ROOT/"scripts"/"fc_control.sh"),*args],
        cwd=str(ROOT),env=env,text=True,capture_output=True,timeout=12
    )
    if cp.returncode!=0:
        raise RuntimeError((cp.stderr or cp.stdout or "команда FC завершилась с ошибкой").strip())
    text=cp.stdout.strip().splitlines()
    if not text: raise RuntimeError("FC не вернул состояние")
    try: return json.loads(text[-1])
    except Exception: raise RuntimeError("Некорректный ответ FC: "+text[-1])

def stop_statustext_monitor():
    global _statustext_proc
    p=_statustext_proc
    _statustext_proc=None
    if p is not None and p.poll() is None:
        try:p.terminate()
        except Exception:pass
        try:p.wait(timeout=2)
        except Exception:
            try:p.kill()
            except Exception:pass

def start_fc_blackbox():
    """Start FC telemetry recorder independently of the flight runtime."""
    global _fc_blackbox_proc,_fc_blackbox_log_handle
    if _fc_blackbox_proc is not None and _fc_blackbox_proc.poll() is None:
        return
    ensure_router()
    log_path=RUN_ROOT/"fc_blackbox_runtime.log"
    _fc_blackbox_log_handle=open_rotating_log(log_path)
    env=os.environ.copy()
    env["MONKEYS_FC"]=FC_ENDPOINT
    _fc_blackbox_proc=subprocess.Popen(
        ["bash",str(ROOT/"scripts"/"run_fc_blackbox.sh")],
        cwd=str(ROOT),env=env,text=True,
        stdout=_fc_blackbox_log_handle,stderr=subprocess.STDOUT,
        start_new_session=True
    )
    time.sleep(0.15)
    if _fc_blackbox_proc.poll() is not None:
        rc=_fc_blackbox_proc.returncode
        _fc_blackbox_proc=None
        if _fc_blackbox_log_handle:
            try:_fc_blackbox_log_handle.close()
            except Exception:pass
            _fc_blackbox_log_handle=None
        raise RuntimeError(f"FC blackbox logger завершился при запуске (code {rc})")
    log_event("INFO","FC blackbox logger запущен: continuous_fc.csv")

def stop_fc_blackbox():
    global _fc_blackbox_proc,_fc_blackbox_log_handle
    p=_fc_blackbox_proc
    _fc_blackbox_proc=None
    if p is not None and p.poll() is None:
        try:os.killpg(p.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            try:p.wait(timeout=1)
            except Exception:pass
    if _fc_blackbox_log_handle:
        try:_fc_blackbox_log_handle.close()
        except Exception:pass
        _fc_blackbox_log_handle=None

def log_runtime_forensic(event,pid=None,detail=""):
    """Append lifecycle boundaries independently of the flight-runtime log."""
    RUN_ROOT.mkdir(parents=True,exist_ok=True)
    p=RUN_ROOT/"runtime_events.csv"
    new=not p.exists()
    with open(p,"a",encoding="utf-8",newline="",buffering=1) as h:
        w=csv.writer(h)
        if new:w.writerow(["mono_ns","wall_ns","event","pid","detail"])
        w.writerow([time.monotonic_ns(),time.time_ns(),event,pid if pid is not None else "",detail])

def _format_console_test_bar(elapsed, duration, width=24):
    elapsed=max(0.0,min(float(duration),float(elapsed)))
    duration=max(0.001,float(duration))
    ratio=elapsed/duration
    filled=max(0,min(width,int(round(width*ratio))))
    bar="█"*filled+"░"*(width-filled)
    def mmss(sec):
        sec=max(0,int(round(sec)))
        return f"{sec//60:02d}:{sec%60:02d}"
    return f"ТЕСТ [{bar}] {mmss(elapsed)} / {mmss(duration)} | осталось {mmss(duration-elapsed)}"

def _start_console_test_statusbar(pid, token, duration, deadline):
    """Показывать реальный таймер bounded-run в консоли Web service, не в OF runtime."""
    if duration <= 0.0 or deadline <= 0.0:
        return
    def statusbar():
        # start_runtime() returns immediately after arming us; give its
        # "АВТОЗАПУСК ГОТОВ" line time to finish before taking one console line.
        time.sleep(0.25)
        last_len=0
        while True:
            with _lock:
                same=(token==_runtime_test_timer_token and running() and _proc is not None and _proc.pid==pid)
            if not same:
                break
            remaining=max(0.0,deadline-time.monotonic())
            elapsed=max(0.0,duration-remaining)
            line=_format_console_test_bar(elapsed,duration)
            print("\r"+line+" "*max(0,last_len-len(line)),end="",flush=True)
            last_len=len(line)
            if remaining <= 0.0:
                break
            time.sleep(min(1.0,remaining))
        # The timer thread can call stop_runtime() and invalidate our token
        # a few milliseconds before this thread renders its final tick.
        # Render 00:00 only when the real deadline has actually elapsed;
        # an early/manual stop must not be presented as a completed test.
        if time.monotonic() >= deadline:
            line=_format_console_test_bar(duration,duration)+" | ЗАВЕРШЁН"
            print("\r"+line+" "*max(0,last_len-len(line)),flush=True)
        else:
            print("",flush=True)
    threading.Thread(target=statusbar,name="runtime-console-statusbar",daemon=True).start()

def _arm_runtime_test_timer(pid):
    """Остановить тот же runtime по истечении тестового лимита."""
    global _runtime_test_timer_token
    with _lock:
        duration=_runtime_test_duration_s
        deadline=_runtime_test_deadline_mono
        _runtime_test_timer_token+=1
        token=_runtime_test_timer_token
    if duration <= 0.0 or deadline <= 0.0:
        return
    _start_console_test_statusbar(pid,token,duration,deadline)
    def timer():
        while True:
            remaining=deadline-time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.25,remaining))
        with _lock:
            same=(token==_runtime_test_timer_token and running() and _proc is not None and _proc.pid==pid)
        if not same:
            return
        log_event("WARN",f"Тестовый таймер истёк: {duration:.0f} с — flight runtime останавливается")
        log_runtime_forensic("RUNTIME_TEST_TIMEOUT",pid,f"duration_s={duration:.3f}")
        try:
            stop_runtime()
        except Exception as e:
            log_event("ERROR","Ошибка автоостановки теста: "+str(e))
    threading.Thread(target=timer,name="runtime-test-timer",daemon=True).start()

def start_runtime(fast_start=False):
    global _proc,_log_handle,_runtime_started_wall,_active_csv
    global _runtime_test_duration_s,_runtime_test_deadline_mono
    with _lock:
        if running():
            log_runtime_forensic("RUNTIME_START_ALREADY_RUNNING",_proc.pid)
            return {"ok":True,"already_running":True,"pid":_proc.pid}
        log_runtime_forensic("RUNTIME_START_REQUEST")
        RUN_ROOT.mkdir(parents=True,exist_ok=True)
        _log_handle=open_rotating_log(WEB_LOG)
        _log_handle.write("\n===== WEB START %s =====\n"%time.strftime("%Y-%m-%d %H:%M:%S"))
        ensure_router()
        env=os.environ.copy()
        env["MONKEYS_LOCAL_GUI"]="0"
        if os.environ.get("MONKEYS_PI4_RUNTIME_SAFE")=="1":
            env["MONKEYS_PI4_RUNTIME_SAFE"]="1"
            env["MONKEYS_PI4_OV5647"]="1"
            env["MONKEYS_NO_LUNA"]="1"
            env["MONKEYS_STABILISED_UNIFIED_PUBLISH"]="0"
            env["MONKEYS_RAW_UNIFIED_PUBLISH"]="0"
        env["MONKEYS_FC"]=FC_ENDPOINT
        # Default Web contour remains Variant B stabilised. Experimental
        # RAW_OF_CONTRACT_V1 is opt-in via the parent environment and requires
        # FC FLOW_OPTIONS=0; never enable both contracts at once.
        raw_contract=str(os.environ.get("MONKEYS_RAW_UNIFIED_PUBLISH","0")).lower() in ("1","true","yes")
        pi4_safe=os.environ.get("MONKEYS_PI4_RUNTIME_SAFE")=="1"
        if pi4_safe:
            env["MONKEYS_STABILISED_UNIFIED_PUBLISH"]="0"
            env["MONKEYS_RAW_UNIFIED_PUBLISH"]="0"
        elif raw_contract:
            env.pop("MONKEYS_STABILISED_UNIFIED_PUBLISH",None)
            env["MONKEYS_RAW_UNIFIED_PUBLISH"]="1"
        else:
            env["MONKEYS_STABILISED_UNIFIED_PUBLISH"]="1"
            env.pop("MONKEYS_RAW_UNIFIED_PUBLISH",None)
        env["MONKEYS_WEB_TELEMETRY_UDP_PORT"]=str(LIVE_UDP_PORT)
        env["MONKEYS_WEB_PREVIEW_PATH"]=str(PREVIEW_PATH)
        try:
            PREVIEW_PATH.unlink()
        except FileNotFoundError:
            pass
        global _live_latest,_live_last_wall,_last_rc_zero_seq,_yaw_zero_deg
        with _lock:
            _live_latest=None
            _live_last_wall=0.0
            _last_rc_zero_seq=None
            _zero["x"]=_zero["y"]=_zero["z"]=None
            _yaw_zero_deg=None
            _raw_zero["n"]=_raw_zero["e"]=None
        _runtime_started_wall=time.time()
        # Optional bounded test run.  Zero/unset keeps normal unlimited runtime.
        # A fast recovery restart inherits the original deadline instead of
        # silently granting the test another full interval.
        if not fast_start or _runtime_test_deadline_mono <= 0.0:
            try:
                _runtime_test_duration_s=max(0.0,float(os.environ.get("MONKEYS_TEST_DURATION_SEC","0") or 0))
            except (TypeError,ValueError):
                _runtime_test_duration_s=0.0
            _runtime_test_deadline_mono=(time.monotonic()+_runtime_test_duration_s) if _runtime_test_duration_s>0.0 else 0.0
        _active_csv=None
        _proc=subprocess.Popen(
            ["bash",str(ROOT/"scripts"/"run_system.sh")],
            cwd=str(ROOT), env=env,
            stdout=_log_handle, stderr=subprocess.STDOUT,
            start_new_session=True, text=True
        )
        pid=_proc.pid
        log_runtime_forensic("RUNTIME_PROCESS_CREATED",pid)
    # Cold start keeps the historical 1.2 s guard so audit/preflight failures
    # surface synchronously. Recovery already uses a verified cached binary:
    # only keep a short spawn guard here; readiness is established separately
    # from fresh runtime telemetry, not by sleeping for 1.2 seconds.
    startup_guard_s=0.12 if fast_start else 1.2
    deadline=time.time()+startup_guard_s
    while time.time()<deadline:
        if _proc.poll() is not None:
            info=runtime_exit_info() or {}
            msg="Flight runtime завершился при запуске"
            if info.get("returncode") is not None:
                msg+=f" (code {info['returncode']})"
            if info.get("log_tail"):
                msg+="\n"+info["log_tail"]
            log_event("ERROR",msg)
            log_runtime_forensic("RUNTIME_START_FAIL",pid,msg.replace("\n"," | "))
            raise RuntimeError(msg)
        time.sleep(0.08)
    log_event("INFO","Flight runtime запущен")
    log_runtime_forensic("RUNTIME_START_OK",pid)
    _arm_runtime_test_timer(pid)
    return {"ok":True,"pid":pid}

def fast_restart_runtime():
    """Перезапустить flight runtime и начать новый локальный сегмент от текущей точки."""
    global _proc,_log_handle,_runtime_started_wall,_active_csv
    global _yaw_zero_deg,_last_rc_zero_seq
    t0=time.monotonic()
    stop_runtime()

    # Recovery starts a NEW local navigation segment.  Do not reset ArduPilot
    # EKF itself: only forget Web/diagnostic origins so the first valid sample
    # from the restarted estimator becomes local (0,0,0).
    with _lock:
        _zero["x"]=_zero["y"]=_zero["z"]=None
        _raw_zero["n"]=_raw_zero["e"]=None
        _imu_zero["n"]=_imu_zero["e"]=_imu_zero["d"]=None
        _fused_zero["n"]=_fused_zero["e"]=None
        _camvc_zero["n"]=_camvc_zero["e"]=None
        _yaw_zero_deg=None
        _last_rc_zero_seq=None
    ws_broadcast({"type":"recovery_segment_reset"})
    log_runtime_forensic("RECOVERY_LOCAL_ZERO_RESET")

    old=os.environ.get("MONKEYS_FAST_RESTART")
    os.environ["MONKEYS_FAST_RESTART"]="1"
    try:
        result=start_runtime(fast_start=True)
    finally:
        if old is None:
            os.environ.pop("MONKEYS_FAST_RESTART",None)
        else:
            os.environ["MONKEYS_FAST_RESTART"]=old
    result["restart_ms"]=round((time.monotonic()-t0)*1000.0,1)

    # Recovery is READY only after the new process has produced fresh,
    # internally usable telemetry.  Do not confuse "process spawned" with
    # "estimator reacquired".
    ready_deadline=time.monotonic()+3.0
    ready=False
    ready_sample=None
    while time.monotonic()<ready_deadline:
        if not running():
            break
        with _lock:
            sample=dict(_live_latest) if isinstance(_live_latest,dict) else None
            sample_wall=_live_last_wall
        if (sample is not None
                and sample_wall >= _runtime_started_wall
                and (time.time()-sample_wall) < 0.25
                and bool(sample.get("worked5_valid",False))
                and bool(sample.get("ekf_valid",False))):
            ready=True
            ready_sample=sample
            break
        time.sleep(0.02)

    result["ready"]=ready
    result["reacquire_ms"]=round((time.monotonic()-t0)*1000.0,1)
    if ready:
        log_runtime_forensic("RUNTIME_FAST_RESTART_READY",result.get("pid"),
                             f"restart_ms={result['restart_ms']};reacquire_ms={result['reacquire_ms']}")
    else:
        log_runtime_forensic("RUNTIME_FAST_RESTART_NOT_READY",result.get("pid"),
                             f"restart_ms={result['restart_ms']};timeout_ms={result['reacquire_ms']}")
    return result

def stop_runtime():
    global _proc,_log_handle,_active_csv,_live_latest,_live_last_wall
    global _runtime_test_duration_s,_runtime_test_deadline_mono,_runtime_test_timer_token
    with _lock:
        if not running():
            # Popen may already have exited while run_system.sh left its
            # process group alive.  Keep the last pid and still terminate the
            # whole group instead of abandoning the flight binary.
            if _proc is None:
                log_runtime_forensic("RUNTIME_STOP_ALREADY_STOPPED")
                return {"ok":True,"already_stopped":True}
            pid=_proc.pid
            log_runtime_forensic("RUNTIME_STOP_STALE_GROUP",pid)
        else:
            pid=_proc.pid
        log_runtime_forensic("RUNTIME_STOP_REQUEST",pid)
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            _proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            try: os.killpg(pid, signal.SIGKILL)
            except ProcessLookupError: pass
            _proc.wait(timeout=2)
        if _log_handle:
            try: _log_handle.close()
            except Exception: pass
            _log_handle=None
        _active_csv=None
        _live_latest=None
        _live_last_wall=0.0
        _runtime_test_duration_s=0.0
        _runtime_test_deadline_mono=0.0
        _runtime_test_timer_token+=1
        ws_broadcast({"type":"runtime","running":False})
        log_event("INFO","Flight runtime остановлен")
        log_runtime_forensic("RUNTIME_STOP_OK",pid)
        return {"ok":True}

def latest_run_csv():
    global _active_csv
    if not running():
        return None
    try:
        # Never expose an old run as live telemetry.  A valid live CSV must
        # have been created/updated after the current runtime was started.
        if _active_csv is not None and _active_csv.exists():
            try:
                if _active_csv.stat().st_mtime >= _runtime_started_wall-1.0:
                    return _active_csv
            except OSError:
                pass
            _active_csv=None

        candidates=[]
        for p in RUN_ROOT.glob("*_OPTICAL_FLOW/optical_flow_mavlink.csv"):
            try:
                if p.stat().st_mtime >= _runtime_started_wall-1.0:
                    candidates.append(p)
            except OSError:
                pass
        candidates.sort(key=lambda p:p.stat().st_mtime,reverse=True)
        if candidates:
            _active_csv=candidates[0]
            return _active_csv
        return None
    except Exception:
        return None

def tail_rows(path, max_rows=180):
    if not path or not path.exists():
        return []
    try:
        with open(path,"rb") as f:
            header_line=f.readline().decode("utf-8","replace").strip()
            if not header_line: return []
            header=next(csv.reader([header_line]))
            f.seek(0,os.SEEK_END)
            size=f.tell()
            back=min(size, 192*1024)
            f.seek(size-back,os.SEEK_SET)
            chunk=f.read().decode("utf-8","replace")
        lines=chunk.splitlines()
        if back < size and lines: lines=lines[1:]
        data=[]
        for line in lines[-max_rows*2:]:
            if not line or line.startswith("mono_ns,"): continue
            try:
                vals=next(csv.reader([line]))
                if len(vals)!=len(header): continue
                data.append(dict(zip(header,vals)))
            except Exception:
                pass
        return data[-max_rows:]
    except Exception:
        return []

def fnum(row,key,default=None):
    try:
        v=float(row.get(key,""))
        return v if math.isfinite(v) else default
    except Exception:
        return default

def telemetry():
    live=running()
    with _lock:
        latest=dict(_live_latest) if _live_latest else None
        age_ms=(time.time()-_live_last_wall)*1000.0 if _live_last_wall else None
    if not latest:
        return {
            "available":False,
            "running":live,
            "transport":"websocket",
            "source_age_ms":age_ms,
            "runtime_exit":runtime_exit_info(),
        }
    latest["running"]=live
    latest["transport"]="websocket"
    latest["source_age_ms"]=age_ms
    return _strip_legacy_imu_fields(latest)

def set_zero():
    global _live_latest,_imu_zero,_fused_zero,_camvc_zero,_yaw_zero_deg
    with _lock:
        if not _live_latest:
            raise RuntimeError("Нет live-телеметрии WebSocket")
        cur=dict(_live_latest)
        forensic_log("ACTION","WEB_HOME_PRESS","BEFORE",sample=cur)
        zx,zy,zz=_zero["x"],_zero["y"],_zero["z"]
        # x_mm/y_mm are in the current HOME body-aligned frame, while
        # _zero[x/y] are FC LOCAL_POSITION_NED North/East coordinates.
        # Rotate the displayed displacement back to N/E before moving HOME.
        local_x=float(cur.get("x_mm",0.0))/1000.0
        local_y=float(cur.get("y_mm",0.0))/1000.0
        old_home_yaw_rad=math.radians(_yaw_zero_deg or 0.0)
        d_n=math.cos(old_home_yaw_rad)*local_x-math.sin(old_home_yaw_rad)*local_y
        d_e=math.sin(old_home_yaw_rad)*local_x+math.cos(old_home_yaw_rad)*local_y
        raw_x=(zx or 0.0)+d_n
        raw_y=(zy or 0.0)+d_e
        raw_z=(zz or 0.0)+float(cur.get("z_mm",0.0))/1000.0
        _zero["x"],_zero["y"],_zero["z"]=raw_x,raw_y,raw_z
        # HOME defines the current heading as local yaw=0.  _live_latest
        # contains yaw relative to the previous HOME, so reconstruct the
        # absolute FC yaw before advancing the baseline.
        cur_yaw_rel=float(cur.get("yaw_deg",0.0) or 0.0)
        _yaw_zero_deg=((_yaw_zero_deg or 0.0)+cur_yaw_rel+180.0)%360.0-180.0
        rn=cur.get("raw_of_n_mm")
        re=cur.get("raw_of_e_mm")
        if rn is not None and re is not None:
            # Current cumulative raw values = previous zero + current relative values.
            _raw_zero["n"]=(_raw_zero["n"] or 0.0)+float(rn)/1000.0
            _raw_zero["e"]=(_raw_zero["e"] or 0.0)+float(re)/1000.0
        # IMU/FUSED telemetry is already in millimetres. Advance each baseline
        # by the currently displayed relative value, exactly like EKF/RAW OF.
        for key,axis in (("imu_dr_n_mm","n"),("imu_dr_e_mm","e"),("imu_dr_d_mm","d")):
            v=cur.get(key)
            if v is not None: _imu_zero[axis]=(_imu_zero[axis] or 0.0)+float(v)
        for key,axis in (("fused_v1_n_mm","n"),("fused_v1_e_mm","e")):
            v=cur.get(key)
            if v is not None: _fused_zero[axis]=(_fused_zero[axis] or 0.0)+float(v)
        for key,axis in (("imu_camvc_n_mm","n"),("imu_camvc_e_mm","e")):
            v=cur.get(key)
            if v is not None: _camvc_zero[axis]=(_camvc_zero[axis] or 0.0)+float(v)
        cur["x_mm"]=cur["y_mm"]=cur["z_mm"]=0.0
        cur["yaw_deg"]=0.0
        cur["ekf_drift_mm"]=0.0
        if rn is not None and re is not None:
            cur["raw_of_n_mm"]=0.0;cur["raw_of_e_mm"]=0.0;cur["raw_of_drift_mm"]=0.0
        cur["imu_dr_n_mm"]=cur["imu_dr_e_mm"]=cur["imu_dr_d_mm"]=0.0
        cur["fused_v1_n_mm"]=cur["fused_v1_e_mm"]=0.0
        cur["imu_camvc_n_mm"]=cur["imu_camvc_e_mm"]=0.0
        cur["imu_camvc_d_mm"]=None
        _strip_legacy_imu_fields(cur)
        _live_latest=cur
        forensic_log("ACTION","WEB_HOME_PRESS","AFTER",sample=cur)
    ws_broadcast({"type":"zero"})

def log_tail(max_lines=120):
    try:
        with open(WEB_LOG,"r",encoding="utf-8",errors="replace") as f:
            return "\n".join(f.readlines()[-max_lines:])
    except Exception:
        return ""

def journal_events():
    with _lock:
        base=list(_journal)
    try:
        with open(WEB_LOG,"r",encoding="utf-8",errors="replace") as f:
            lines=f.readlines()[-420:]
        for line in lines:
            t=line.rstrip()
            if not t: continue
            base.append({"ts":"","level":"RUNTIME","text":t})
    except Exception:
        pass
    return base[-500:]

HTML=r'''<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>monkeysStab — UAV Control & Visualizer</title>
<style>
*{box-sizing:border-box}
:root{
 --bg:#07111b;--panel:#0c1a27;--panel2:#0a1621;--line:#1d3950;--line2:#14293a;
 --text:#e9f3fb;--muted:#87a5bd;--blue:#1299ff;--green:#0bd777;--red:#ff4654;
 --yellow:#ffc928;--cyan:#15d2ff;--purple:#c98bff;
 font-family:Inter,ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;
 color:var(--text);background:var(--bg)
}
html,body{margin:0;min-height:100%;background:
 radial-gradient(circle at 75% -10%,#0e2638 0,#07111b 35%),
 linear-gradient(#07111b,#061018)}
body{overflow-x:hidden}
button,input,select{font:inherit}
button{cursor:pointer}
.topbar{height:64px;border-bottom:1px solid var(--line);display:flex;align-items:center;padding:0 18px;background:#071522dd;backdrop-filter:blur(12px);position:sticky;top:0;z-index:20}
.brand{display:flex;align-items:center;gap:12px;min-width:280px}
.logo{width:34px;height:34px;border:3px solid var(--blue);transform:rotate(30deg);border-radius:8px;position:relative;box-shadow:0 0 18px #1299ff55}
.logo:after{content:"";position:absolute;inset:7px;border:2px solid #42b6ff;border-radius:4px}
.brand b{font-size:22px;letter-spacing:.3px}.brand small{display:block;color:#9ab3c8;margin-top:1px}
.nav{display:flex;height:100%;align-items:center;gap:8px;flex:1}
.nav button{height:100%;padding:0 22px;background:none;color:#a4bdd1;border:0;border-bottom:3px solid transparent;font-weight:700}
.nav button.active{color:#20aaff;border-bottom-color:#20aaff;background:#0f2d44}
.topStatus{display:flex;gap:22px;align-items:center;font-size:13px;color:#9fb5c6}
.okdot{width:10px;height:10px;border-radius:50%;display:inline-block;background:#17d768;box-shadow:0 0 10px #17d76888;margin-right:7px}
.baddot{background:#ff4b59;box-shadow:0 0 10px #ff4b5988}
.main{display:grid;grid-template-columns:275px minmax(580px,1fr) 270px;gap:12px;padding:12px;max-width:1900px;margin:auto}
.col{display:flex;flex-direction:column;gap:12px}
.card{background:linear-gradient(180deg,#0c1a27,#091621);border:1px solid #1c3b53;border-radius:9px;box-shadow:inset 0 1px 0 #ffffff08;padding:13px}
.card h3{font-size:15px;margin:0 0 12px}.card h4{font-size:13px;color:#a8bfd0;margin:8px 0}
.fcstate{font-size:25px;font-weight:800;display:flex;align-items:center;gap:10px}.modeLine{margin:7px 0 10px;color:#a7bfd1}.modeLine b{color:#27aaff}
.btnrow{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px}
.btnrow3{display:grid;grid-template-columns:1fr;gap:7px;margin-top:9px}
.btnrow3 .btn{width:100%;min-width:0;padding:9px 6px;font-size:12px}
.btn{border:1px solid #31526a;border-radius:6px;padding:10px 8px;background:#102438;color:#eaf5fc;font-weight:800;min-width:0;max-width:100%;overflow:hidden;text-overflow:ellipsis}
.btn:hover{filter:brightness(1.12)}.btn.green{background:#0aad5b;border-color:#0de479}.btn.red{background:#a92431;border-color:#ff4050}.btn.blue{background:#087aca;border-color:#17a9ff}.btn.stop{color:#ff5361;background:#24111a;border-color:#dc3444}
.field{display:grid;grid-template-columns:1fr 112px;align-items:center;gap:8px;margin:9px 0;color:#9fb7ca;font-size:13px}
.field input,.field select{width:100%;background:#0b1823;color:#eaf3f9;border:1px solid #26465d;border-radius:5px;padding:7px}
.startBig{width:100%;margin-top:10px;border:1px solid #12df77;background:#0b9f56;color:white;border-radius:6px;padding:11px;font-weight:900}
.stopBig{width:100%;margin-top:8px;border:1px solid #ff4352;background:#201018;color:#ff5a67;border-radius:6px;padding:10px;font-weight:900}
.kv{display:grid;grid-template-columns:1fr auto;gap:5px 9px;font-size:12px}.kv span:nth-child(odd){color:#86a7bf}.kv span:nth-child(even){color:#dfeaf2}
.sceneCard{padding:0;overflow:hidden;position:relative;min-height:625px}
.sceneTitle{position:absolute;left:14px;top:10px;z-index:4;font-weight:800}
.voPreviewCard{padding:0;overflow:hidden}
.voPreviewHead{display:flex;align-items:center;justify-content:space-between;padding:10px 13px;border-bottom:1px solid var(--line2)}
.voPreviewHead h3{margin:0}.voPreviewHead span{font-size:11px;color:var(--muted)}
.voPreviewWrap{position:relative;background:#02070b;aspect-ratio:4/3;max-height:360px;display:flex;align-items:center;justify-content:center}
#voPreview{display:block;width:100%;height:100%;object-fit:contain}
.voPreviewLegend{position:absolute;left:9px;bottom:8px;padding:4px 7px;border-radius:4px;background:#07121acc;font-size:11px;color:#cfe2ef}
.voPreviewLegend b{color:#31ef79}
#glCanvas{display:block;width:100%;height:625px;background:
 radial-gradient(circle at 50% 15%,#11304a55,#07121c 52%),#07121c}
.modelThumb{position:absolute;inset:10px 18px 18px;pointer-events:none}
.modelThumb:before,.modelThumb:after{content:"";position:absolute;left:50%;top:50%;width:66px;height:5px;background:#8ba6b8;border-radius:4px;transform-origin:center}
.modelThumb:before{transform:translate(-50%,-50%) rotate(28deg)}
.modelThumb:after{transform:translate(-50%,-50%) rotate(-28deg)}
.modelThumb i{position:absolute;left:50%;top:50%;width:28px;height:16px;border-radius:50%;background:#52a8d8;transform:translate(-50%,-50%);box-shadow:-30px -15px 0 -5px #9abdce,30px -15px 0 -5px #9abdce,-30px 15px 0 -5px #9abdce,30px 15px 0 -5px #9abdce}
.viewItem{position:relative;overflow:hidden}
.viewItem span{position:absolute;left:0;right:0;bottom:4px;text-align:center;z-index:3;text-shadow:0 1px 3px #000;background:#07121aaa;padding:2px 0}
.sceneControls{position:absolute;right:12px;top:10px;background:#081521dd;border:1px solid #26475e;border-radius:7px;padding:8px 10px;font-size:12px;z-index:4}
.sceneControls label{display:block;margin:5px 0;color:#b3c9d7}
.sceneLegend{position:absolute;left:14px;bottom:12px;display:flex;gap:8px;z-index:4}
.miniBtn{border:1px solid #294c65;background:#0c1c29;color:#dbe8f0;padding:8px 11px;border-radius:5px}
.telemetryStrip{position:absolute;right:14px;bottom:12px;background:#081521cc;border:1px solid #24445a;border-radius:6px;padding:8px 11px;font-size:12px;color:#a9c0d0;z-index:4}
.metrics{display:grid;grid-template-columns:repeat(5,1fr);gap:8px;margin-top:10px}
.compareCard{margin-top:10px;padding:12px}
.compareHead{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:8px}
.compareHead h3{margin:0}.compareHint{font-size:11px;color:var(--muted)}
#motionCompare{display:block;width:100%;height:430px;background:#06111a;border:1px solid #17364b;border-radius:7px}
.compareLegend{display:grid;grid-template-columns:repeat(4,1fr);gap:7px;margin-top:8px}
.compareItem{background:#081722;border:1px solid #17364b;border-radius:6px;padding:8px}
.compareItem span{display:block;font-size:11px;color:#8da9bd}.compareItem b{font-size:15px}.compareDot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px}
@media(max-width:850px){.compareLegend{grid-template-columns:1fr 1fr}#motionCompare{height:340px}}
.metric{background:#09151f;border:1px solid #163147;border-radius:7px;padding:9px}.metric span{font-size:11px;color:#7fa1ba}.metric b{display:block;font-size:18px;margin-top:2px}
.gaugeBox{padding:10px 12px}.gLine{display:grid;grid-template-columns:58px 1fr 52px;gap:7px;align-items:center;margin:12px 0;font-size:12px}.gLine strong{text-align:right}
.bar{height:7px;background:#19364a;border-radius:10px;position:relative}.bar:after{content:"";position:absolute;left:50%;top:-5px;height:17px;width:2px;background:#5f7f95}.needle{position:absolute;top:-4px;width:5px;height:15px;border-radius:2px;background:#18e278;box-shadow:0 0 8px currentColor;transform:translateX(-50%)}
#compass{width:100%;height:205px;display:block}
.viewGrid{display:grid;grid-template-columns:1fr 1fr;gap:7px}.viewItem{height:80px;border:1px solid #294b63;border-radius:5px;background:#08141e;display:flex;align-items:end;justify-content:center;padding:6px;color:#9fb9cb;font-size:11px}.viewItem.active{border-color:#18a8ff;box-shadow:inset 0 0 0 1px #18a8ff55}
.bottomCharts{display:grid;grid-template-columns:1.3fr 1fr 1fr;gap:10px;margin-top:10px}.chartCard{padding:10px}.chartCard h3{margin-bottom:4px}.chart{width:100%;height:170px;display:block;background:#08131d;border-radius:5px}
.logCard{grid-column:1/-1}.logHead{display:flex;justify-content:space-between;align-items:center}.log{height:120px;background:#07121a;border:1px solid #122a3c;border-radius:5px;padding:8px;overflow:auto;white-space:pre-wrap;font:12px/1.35 ui-monospace,SFMono-Regular,Consolas,monospace;color:#9cb5c8}
.appView{display:none}.appView.activeView{display:block}
.viewPage{max-width:1500px;margin:auto;padding:16px}
.pageGrid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.tabs{display:flex;gap:7px;margin-bottom:12px}.tabBtn{border:1px solid #294b63;background:#0b1b28;color:#9fbbcf;padding:9px 15px;border-radius:6px;font-weight:800}.tabBtn.active{background:#0d73b7;color:white;border-color:#20a9ff}
.tabPane{display:none}.tabPane.active{display:block}
.paramTable{width:100%;border-collapse:collapse;font-size:13px}.paramTable th,.paramTable td{border-bottom:1px solid #173247;padding:8px;text-align:left}.paramTable th{color:#8eacc1}.paramTable input{width:140px;background:#07141e;color:#e9f3fb;border:1px solid #2a4b62;border-radius:5px;padding:7px}
.actionBar{display:flex;gap:8px;margin:10px 0;flex-wrap:wrap}
.systemModes{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}.systemMode{border:1px solid #284a61;background:#081620;padding:18px;border-radius:9px;cursor:pointer}.systemMode.active{border-color:#18a8ff;box-shadow:inset 0 0 0 1px #18a8ff55}.systemMode h3{margin-top:0}.systemMode p{color:#8da9bd;font-size:13px}
.eventList{height:620px;overflow:auto;background:#07121a;border:1px solid #173247;border-radius:6px}.eventRow{display:grid;grid-template-columns:90px 70px 1fr;gap:10px;padding:7px 10px;border-bottom:1px solid #10283a;font:12px/1.4 ui-monospace,monospace}.eventRow .ts{color:#7296af}.eventRow .INFO{color:#1faaff}.eventRow .WARN{color:#ffc52d}.eventRow .ERROR{color:#ff5865}
.footer{height:38px;border-top:1px solid #16364d;display:flex;align-items:center;justify-content:space-between;padding:0 16px;color:#7798ae;font-size:12px}
.badge{display:inline-flex;align-items:center;gap:5px}
@media(max-width:1250px){.main{grid-template-columns:255px 1fr}.right{grid-column:1/-1;display:grid;grid-template-columns:1fr 1fr 1fr}.sceneCard{min-height:520px}#glCanvas{height:520px}.bottomCharts{grid-template-columns:1fr}.topStatus{display:none}}
@media(max-width:850px){.main{grid-template-columns:1fr}.left,.right{grid-column:auto}.right{display:flex}.nav{display:none}.brand{min-width:0;flex:1}.sceneCard{min-height:430px}#glCanvas{height:430px}.metrics{grid-template-columns:repeat(2,1fr)}}

/* IMU DR display intentionally disabled: production UI is WORKED5 + EKF3. */
.metrics:has(#imuX),
.card:has(#imuAcc),
.metrics:has(#camvcX),
.metrics:has(#fusedX),
.compareItem:has(#cmpImu),
.compareItem:has(#cmpFused) { display:none !important; }
</style>
</head>
<body>
<div class="topbar">
 <div class="brand"><div class="logo"></div><div><b>monkeysStab</b><small>UAV Control & Visualizer</small></div></div>
 <div class="nav">
  <button class="active" onclick="showView('flight',this)">▲ ПОЛЁТ</button><button onclick="showView('settings',this)">⚙ НАСТРОЙКИ</button><button onclick="showView('telemetry',this)">∿ ТЕЛЕМЕТРИЯ</button><button onclick="showView('journal',this)">▤ ЖУРНАЛ</button><button onclick="showView('system',this)">⚙ СИСТЕМА</button>
 </div>
 <div class="topStatus">
  <div id="testTimerBox" style="display:none;min-width:220px">
   <div style="display:flex;justify-content:space-between;gap:10px;margin-bottom:3px;font-size:11px"><b>ТЕСТ 30 МИН</b><strong id="testTimerText">30:00</strong></div>
   <div style="height:7px;border:1px solid var(--line);border-radius:999px;overflow:hidden;background:#07111b"><div id="testTimerBar" style="height:100%;width:0%;background:var(--blue);transition:width .25s linear"></div></div>
  </div>
  <span><i id="linkDot" class="okdot baddot"></i>СВЯЗЬ: <b id="linkText">НЕТ</b></span><span id="clock">--:--:--</span>
 </div>
</div>

<section id="view-flight" class="appView activeView"><div class="main">
 <div class="col left">
  <div class="card">
   <h3>Полётный контроллер</h3>
   <div class="fcstate"><i id="fcDotBig" class="okdot baddot"></i><span id="fcState">НЕТ СВЯЗИ</span></div>
   <div class="modeLine">Режим: <b id="fcMode">—</b></div>
   <div class="btnrow"><button class="btn green" onclick="armFc()">🔒 ARM</button><button class="btn red" onclick="disarmFc()">🔒 DISARM</button></div>
   <div class="btnrow3">
    <button id="mStab" class="btn" onclick="setMode('stabilize','Stabilize')">STABILIZE</button>
    <button id="mPos" class="btn" onclick="setMode('poshold','PosHold')">POSHOLD</button>
   </div>
   <div class="field" style="margin-top:9px"><span>Высота TAKEOFF, м</span><input id="takeoffAlt" type="number" min="0.10" max="10.0" step="0.05" value="0.30"></div>
   <div class="btnrow">
    <button id="takeoffBtn" class="btn green" onclick="takeoffFc()">▲ TAKEOFF</button>
    <button id="landBtn" class="btn red" onclick="landFc()">▼ LAND</button>
   </div>
   <div id="fcMsg" style="margin-top:8px;color:#7798ae;font-size:11px">TAKEOFF: Guided + относительная высота. LAND: режим Land.</div>
  </div>

  <div class="card">
   <h3>Стартовые параметры</h3>
   <div class="field"><span>Focal scale</span><input id="focal" type="number" min=".5" max="2" step=".001"></div>
   <div class="field"><span>ROI x0</span><input id="r0" type="number" step=".01"></div>
   <div class="field"><span>ROI y0</span><input id="r1" type="number" step=".01"></div>
   <div class="field"><span>ROI x1</span><input id="r2" type="number" step=".01"></div>
   <div class="field"><span>ROI y1</span><input id="r3" type="number" step=".01"></div>
   <div class="field"><span>Feature points</span><input id="features" type="number" min="100" max="1000" step="10"></div>
   <button class="btn" style="width:100%" onclick="saveConfig()">СОХРАНИТЬ ПАРАМЕТРЫ</button>
   <div id="saveMsg" style="font-size:11px;color:#7798ae;margin-top:5px"></div>
   <button class="startBig" onclick="start()">▶ ЗАПУСТИТЬ СИСТЕМУ</button>
   <button class="stopBig" onclick="stop()">■ ОСТАНОВИТЬ</button>
   <button id="restartEstimatorBtn" class="btn" style="width:100%;margin-top:8px" onclick="restartEstimator()">↻ ПЕРЕЗАПУСТИТЬ ОЦЕНКУ / НОВЫЙ НОЛЬ</button>
   <button id="runRecordBtn" class="btn blue" style="width:100%;margin-top:8px" onclick="toggleRunRecord()">● ЗАПИСЬ ПРОГОНА</button>
   <div id="runtimeError" style="display:none;margin-top:8px;padding:8px;border:1px solid #8b3038;border-radius:5px;background:#271018;color:#ff7b86;font:11px/1.35 ui-monospace,monospace;white-space:pre-wrap;max-height:180px;overflow:auto"></div>
  </div>

  <div class="card">
   <h3>Текущая геометрия (мм)</h3>
   <div id="geomKv" class="kv"></div>
  </div>
  <div class="card">
   <h3>Профиль FC (EKF3)</h3>
   <div id="profileKv" class="kv"></div>
  </div>
 </div>

 <div class="col center">
  <div class="card sceneCard">
   <div class="sceneTitle">3D — FC EKF · сетка 5 см</div>
   <canvas id="glCanvas"></canvas><canvas id="lightCanvas" style="display:none;width:100%;height:625px;background:#07121c"></canvas>
   <div class="sceneControls">
    <label><input id="showTrail" type="checkbox" checked> Траектория</label>
    <label><input id="showGrid" type="checkbox" checked> Сетка</label>
    <label><input id="showAxes" type="checkbox" checked> Оси X/Y/Z</label>
    <label><input id="followCam" type="checkbox"> След камеры</label>
   </div>
   <div class="sceneLegend"><button class="miniBtn" onclick="zero()">⟳ HOME = текущая точка</button><button class="miniBtn" onclick="resetView()">⌂ Сброс вида</button></div>
   <div class="telemetryStrip"><span id="sceneXYZ">X 0.000 · Y 0.000 · Z 0.000 m</span></div>
  </div>

  <div class="card voPreviewCard">
   <div class="voPreviewHead"><h3>VO — камера / точки захвата</h3><span id="voPreviewState">нет кадра</span></div>
   <div class="voPreviewWrap">
    <img id="voPreview" alt="OV9281 VO preview">
    <div class="voPreviewLegend"><b>●</b> RANSAC inliers · жёлтая рамка = feature ROI</div>
   </div>
  </div>

  <div class="metrics">
   <div class="metric"><span>X · FC EKF</span><b id="mx">—</b></div>
   <div class="metric"><span>Y · FC EKF</span><b id="my">—</b></div>
   <div class="metric"><span>Z · FC EKF</span><b id="mz">—</b></div>
   <div class="metric"><span>TF-Luna</span><b id="mr">—</b></div>
   <div class="metric"><span>RAW OF</span><b id="mq">—</b></div>
  </div>
  <div class="metrics">
   <div class="metric"><span>X · IMU DR</span><b id="imuX">—</b></div>
   <div class="metric"><span>Y · IMU DR</span><b id="imuY">—</b></div>
   <div class="metric"><span>Z · IMU DR</span><b id="imuZ">—</b></div>
   <div class="metric"><span>Источник</span><b>HIGHRES IMU</b></div>
   <div class="metric"><span>Stationary</span><b id="imuStationary">—</b></div>
  </div>
  <div class="card" style="margin-top:8px">
   <h3>IMU DR — диагностика покоя / интегрирования</h3>
   <div class="kv" style="grid-template-columns:150px 1fr 150px 1fr 150px 1fr">
    <span>ACC N / E / D</span><span id="imuAcc">—</span>
    <span>VEL N / E / D</span><span id="imuVel">—</span>
    <span>dt</span><span id="imuDt">—</span>
    <span>|a| / ACC gate</span><span id="imuAmag">—</span>
    <span>|gyro| / GYRO gate</span><span id="imuGmag">—</span>
    <span>stationary samples</span><span id="imuStatSamples">—</span>
    <span>ACC rejects</span><span id="imuAccRejects">—</span>
    <span>GYRO rejects</span><span id="imuGyroRejects">—</span>
    <span>CAM stationary</span><span id="imuCamStat">—</span>
   </div>
  </div>

  <div class="metrics">
   <div class="metric"><span>X · IMU+CAM ZUPT</span><b id="camvcX">—</b></div>
   <div class="metric"><span>Y · IMU+CAM ZUPT</span><b id="camvcY">—</b></div>
   <div class="metric"><span>Z · CAM constraint</span><b id="camvcZ">—</b></div>
   <div class="metric"><span>V N/E</span><b id="camvcVel" style="font-size:12px">—</b></div>
   <div class="metric"><span>XY ZUPT state</span><b id="camvcState">—</b></div>
  </div>

  <div class="metrics">
   <div class="metric"><span>X · FC EKF</span><b id="camX">—</b></div>
   <div class="metric"><span>Y · FC EKF</span><b id="camY">—</b></div>
   <div class="metric"><span>Z · FC EKF</span><b id="camZ">—</b></div>
   <div class="metric"><span>Источник</span><b>FC EKF</b></div>
   <div class="metric"><span>Примечание</span><b style="font-size:12px">Диагностика положения аппарата</b></div>
  </div>
  <div class="metrics">
   <div class="metric"><span>X · ОБЩЕЕ</span><b id="fusedX">—</b></div>
   <div class="metric"><span>Y · ОБЩЕЕ</span><b id="fusedY">—</b></div>
   <div class="metric"><span>Z · ОБЩЕЕ</span><b id="fusedZ">—</b></div>
   <div class="metric"><span>Источник</span><b>FUSED V1</b></div>
   <div class="metric"><span>Состояние</span><b id="fusedState">—</b></div>
  </div>

  <div class="card compareCard">
   <div class="compareHead">
    <h3>Перемещение — FC EKF · клетка 5 см</h3>
    <span class="compareHint">вид сверху · HOME X/Y · realtime · клетка 5 см</span>
   </div>
   <canvas id="motionCompare"></canvas>
   <div class="compareLegend">
    <div class="compareItem" style="display:none"><span>WORKED5</span><b id="cmpCam">—</b></div>
    <div class="compareItem" style="display:none"><span>IMU DR</span><b id="cmpImu">—</b></div>
    <div class="compareItem" style="display:none"><span>FUSED V1</span><b id="cmpFused">—</b></div>
    <div class="compareItem"><span><i class="compareDot" style="background:#0bd777"></i>FC EKF</span><b id="cmpEkf">—</b></div>
   </div>
  </div>

  <div class="card" style="margin-top:10px">
   <h3>Дрейф от HOME — EKF vs RAW Optical Flow</h3>
   <div class="kv" style="grid-template-columns:155px 1fr 155px 1fr 155px 1fr">
    <span>EKF ΔN / ΔE</span><span id="ekfNE">—</span>
    <span>EKF |XY|</span><span id="ekfDrift">—</span>
    <span>EKF vN / vE</span><span id="ekfVel">—</span>
    <span>RAW OF ΔN / ΔE</span><span id="rawNE">—</span>
    <span>RAW OF |XY|</span><span id="rawDrift">—</span>
    <span>RAW OF vN / vE</span><span id="rawVel">—</span>
   </div>
  </div>

  <div class="bottomCharts">
   <div class="card chartCard"><h3>X / Y / Z (м)</h3><canvas id="xyzChart" class="chart"></canvas></div>
   <div class="card chartCard"><h3>Скорость (м/с)</h3><canvas id="speedChart" class="chart"></canvas></div>
   <div class="card chartCard"><h3>TF-Luna (м)</h3><canvas id="rangeChart" class="chart"></canvas></div>
   <div class="card logCard">
    <div class="logHead"><h3>Messages FC / Mission Planner</h3><span style="color:#7898ae;font-size:11px">MAVLink STATUSTEXT</span></div>
    <div id="log" class="log"></div>
   </div>
  </div>
 </div>

 <div class="col right">
  <div class="card gaugeBox">
   <h3>Инклинометр</h3>
   <div class="gLine"><span>ROLL</span><div class="bar"><i id="rollNeedle" class="needle"></i></div><strong id="roll">—</strong></div>
   <div class="gLine"><span>PITCH</span><div class="bar"><i id="pitchNeedle" class="needle"></i></div><strong id="pitch">—</strong></div>
   <div class="gLine"><span>YAW</span><div class="bar"><i id="yawNeedle" class="needle" style="background:#1ca6ff"></i></div><strong id="yaw">—</strong></div>
  </div>
  <div class="card"><h3>Компас (Yaw)</h3><canvas id="compass" width="240" height="205"></canvas></div>
  <div class="card">
   <h3>Виды модели</h3>
   <div class="viewGrid">
    <div class="viewItem active" onclick="setView('iso',this)"><div class="modelThumb"><i></i></div><span>Изометрия</span></div>
    <div class="viewItem" onclick="setView('side',this)"><div class="modelThumb" style="transform:scaleY(.55)"><i></i></div><span>Сбоку</span></div>
    <div class="viewItem" onclick="setView('front',this)"><div class="modelThumb" style="transform:scaleX(.65)"><i></i></div><span>Спереди</span></div>
    <div class="viewItem" onclick="setView('top',this)"><div class="modelThumb" style="transform:rotate(45deg)"><i></i></div><span>Сверху</span></div>
   </div>
  </div>
  <div class="card">
   <h3>Состояние</h3>
   <div class="kv">
    <span>Runtime</span><span id="runState">Остановлен</span>
    <span>FC</span><span id="arm">—</span>
    <span>Inliers</span><span id="inl">—</span>
    <span>Frame</span><span id="frame">—</span>
    <span>EKF</span><span id="ekf">—</span>
   </div>
  </div>
 </div>
</div></section>

<section id="view-settings" class="appView">
 <div class="viewPage">
  <h2>Настройки</h2>
  <div class="tabs">
   <button class="tabBtn active" onclick="showSettingsTab('runtime',this)">Стартовые параметры</button>
   <button class="tabBtn" onclick="showSettingsTab('fc',this)">Параметры FC</button>
   <button class="tabBtn" onclick="showSettingsTab('geometry',this)">Геометрия</button>
  </div>
  <div id="settings-runtime" class="tabPane active">
   <div class="card" style="max-width:720px">
    <h3>Параметры Optical Flow runtime</h3>
    <div class="field"><span>Focal scale</span><input id="sfocal" type="number" min=".5" max="2" step=".001"></div>
    <div class="field"><span>ROI x0</span><input id="sr0" type="number" step=".01"></div>
    <div class="field"><span>ROI y0</span><input id="sr1" type="number" step=".01"></div>
    <div class="field"><span>ROI x1</span><input id="sr2" type="number" step=".01"></div>
    <div class="field"><span>ROI y1</span><input id="sr3" type="number" step=".01"></div>
    <div class="field"><span>Feature points</span><input id="sfeatures" type="number" min="100" max="1000"></div>
    <button class="btn blue" onclick="saveRuntimeSettings()">СОХРАНИТЬ</button>
    <span id="runtimeSettingsMsg" style="margin-left:8px;color:#86a7bf"></span>
   </div>
  </div>
  <div id="settings-fc" class="tabPane">
   <div class="card">
    <h3>Критические параметры FC</h3>
    <p style="color:#89a7bc">Читаются непосредственно из ArduPilot. Запись разрешена только при DISARMED и остановленном flight runtime.</p>
    <div class="actionBar"><button class="btn blue" onclick="loadFcParams()">ПРОЧИТАТЬ ИЗ FC</button><button class="btn green" onclick="writeFcParams()">ЗАПИСАТЬ И ПРОВЕРИТЬ</button></div>
    <table class="paramTable"><thead><tr><th>Параметр</th><th>FC</th><th>Новое значение</th></tr></thead><tbody id="fcParamRows"></tbody></table>
    <div id="fcParamMsg" style="margin-top:10px;color:#86a7bf"></div>
   </div>
  </div>
  <div id="settings-geometry" class="tabPane">
   <div class="card" style="max-width:900px">
    <h3>Геометрия OV9281 + TF-Luna</h3>
    <p style="color:#89a7bc">FRD относительно центра IMU: X вперёд, Y вправо, Z вниз. Ввод в миллиметрах.</p>
    <div class="actionBar"><button class="btn blue" onclick="loadGeometry()">ПРОЧИТАТЬ ИЗ FC</button><button class="btn green" onclick="writeGeometry()">ЗАПИСАТЬ В FC И CONFIG</button></div>
    <table class="paramTable"><thead><tr><th>Датчик</th><th>X, мм</th><th>Y, мм</th><th>Z, мм</th></tr></thead>
     <tbody>
      <tr><td>OV9281</td><td><input id="gx0"></td><td><input id="gy0"></td><td><input id="gz0"></td></tr>
      <tr><td>TF-Luna</td><td><input id="grx"></td><td><input id="gry"></td><td><input id="grz"></td></tr>
     </tbody>
    </table>
    <div id="geometryMsg" style="margin-top:10px;color:#86a7bf"></div>
   </div>
  </div>
 </div>
</section>

<section id="view-telemetry" class="appView">
 <div class="viewPage">
  <h2>Телеметрия</h2>
  <div class="metrics">
   <div class="metric"><span>X</span><b id="tmx">—</b></div><div class="metric"><span>Y</span><b id="tmy">—</b></div><div class="metric"><span>Z</span><b id="tmz">—</b></div><div class="metric"><span>TF-Luna</span><b id="tmr">—</b></div><div class="metric"><span>Quality</span><b id="tmq">—</b></div>
  </div>
  <div class="pageGrid" style="margin-top:12px">
   <div class="card"><h3>X / Y / Z</h3><canvas id="tXyzChart" class="chart" style="height:300px"></canvas></div>
   <div class="card"><h3>Скорость</h3><canvas id="tSpeedChart" class="chart" style="height:300px"></canvas></div>
   <div class="card"><h3>TF-Luna</h3><canvas id="tRangeChart" class="chart" style="height:300px"></canvas></div>
   <div class="card"><h3>Ориентация</h3><div class="kv"><span>Roll</span><span id="troll">—</span><span>Pitch</span><span id="tpitch">—</span><span>Yaw</span><span id="tyaw">—</span><span>Inliers</span><span id="tinl">—</span><span>EKF</span><span id="tekf">—</span></div></div>
  </div>
 </div>
</section>

<section id="view-journal" class="appView">
 <div class="viewPage">
  <h2>Журнал Web UI / Runtime</h2>
  <p style="color:#86a7bf">До 500 последних событий приложения. Сообщения ArduPilot STATUSTEXT находятся на view «Полёт».</p>
  <div class="actionBar" style="margin-bottom:12px"><button class="btn blue" onclick="openLogsModal()">СКАЧАТЬ ЛОГИ</button></div>
  <div id="journalEvents" class="eventList"></div>
 </div>
</section>

<div id="runNameModal" style="display:none;position:fixed;inset:0;background:#000a;z-index:100;align-items:center;justify-content:center">
 <div class="card" style="width:min(520px,92vw);padding:18px">
  <h3 style="margin-top:0">Название прогона</h3>
  <input id="runNameInput" type="text" maxlength="100" placeholder="Например: X+500_return_01" style="width:100%;margin:10px 0">
  <div id="runNameMsg" style="color:#ff7b86;font-size:12px;min-height:18px"></div>
  <div class="actionBar"><button class="btn green" onclick="saveRunName()">СОХРАНИТЬ ПРОГОН</button></div>
 </div>
</div>

<div id="logsModal" style="display:none;position:fixed;inset:0;background:#000a;z-index:100;align-items:center;justify-content:center">
 <div class="card" style="width:min(720px,94vw);max-height:80vh;overflow:auto;padding:18px">
  <h3 style="margin-top:0">Сохранённые прогоны</h3>
  <div id="logsList"></div>
  <div class="actionBar" style="margin-top:14px">
   <button class="btn green" onclick="downloadSelectedLogs()">СКАЧАТЬ ВЫБРАННЫЕ</button>
   <button class="btn" onclick="closeLogsModal()">ЗАКРЫТЬ</button>
  </div>
 </div>
</div>

<section id="view-system" class="appView">
 <div class="viewPage">
  <h2>Система</h2>
  <div class="card">
   <h3>Режим визуализации</h3>
   <div class="systemModes">
    <div id="vmSimple" class="systemMode" onclick="setVisualizationMode('simple')"><h3>Упрощённое 3D</h3><p>Текущий лёгкий WebGL-каркас, сетка, траектория и ориентация.</p></div>
    <div id="vmLight" class="systemMode" onclick="setVisualizationMode('light')"><h3>Лёгкое 2D</h3><p>Без 3D. Только XY-график, траектория и числовые показатели.</p></div>
   </div>
   <div id="visualModeMsg" style="margin-top:12px;color:#86a7bf"></div>
  </div>
  <div class="card" style="margin-top:12px">
   <h3>ChArUco для калибровки камеры</h3>
   <div class="kv">
    <span>Доска</span><span>5 x 7 квадратов</span>
    <span>Размер квадрата</span><span>30.0 мм</span>
    <span>Размер маркера</span><span>22.0 мм</span>
    <span>Словарь</span><span>DICT_4X4_50</span>
    <span>Формат</span><span>A4 PDF</span>
   </div>
   <p style="color:#8da9bd;font-size:13px">Печатать строго 100% / Actual size, без Fit to page. После печати проверьте контрольный отрезок 100 мм.</p>
   <div class="actionBar">
    <button class="btn blue" onclick="window.open('/charuco.pdf','_blank')">ОТКРЫТЬ CHARUCO.PDF</button>
    <a class="btn green" href="/charuco.pdf?download=1" download="charuco.pdf" style="text-decoration:none;display:inline-block">СКАЧАТЬ CHARUCO.PDF</a>
   </div>
  </div>
  <div class="card" style="margin-top:12px"><h3>Сеть</h3><div class="kv"><span>MAVLink router</span><span>TCP 127.0.0.1:5760</span><span>Mission Planner</span><span>UDP 14550</span><span>Web UI</span><span>TCP 8080</span></div></div>
 </div>
</section>

<div class="footer">
 <div>● &nbsp; monkeysStab Web UI &nbsp; | &nbsp; Raspberry Pi 5</div>
 <div>MAVLink router: <b id="routerStatus" style="color:#16d979">OK</b> &nbsp; | &nbsp; Mission Planner: UDP 14550 &nbsp; | &nbsp; Runtime: <b id="footerRuntime">остановлен</b></div>
</div>

<script>
const $=id=>document.getElementById(id);
let latest=null,fcLatest=null,lastChartPaint=0;
let telemetryWs=null,wsReconnectTimer=null,wsHistory=[],wsTrail=[],wsT0=null;
let motionTrails={cam:[],imu:[],fused:[],ekf:[]};
const MOTION_MAX_POINTS=900;
const WS_MAX_POINTS=300;
let viewMode='iso',viewYaw=.75,viewPitch=.65,viewDist=6.4;
let drag=false,lastX=0,lastY=0;

async function api(path,opt){let r=await fetch(path,opt);let j=await r.json();if(!r.ok)throw new Error(j.error||r.statusText);return j}
function showView(id,btn){
 document.querySelectorAll('.appView').forEach(x=>x.classList.remove('activeView'));
 let v=$('view-'+id);if(v)v.classList.add('activeView');
 document.querySelectorAll('.nav button').forEach(x=>x.classList.remove('active'));
 if(btn)btn.classList.add('active');
 window.scrollTo({top:0,behavior:'instant'});
 if(id==='settings'){loadFcParams();loadGeometry()}
 if(id==='journal')refreshJournal();
 if(id==='system')refreshVisualizationMode();
 if(id==='telemetry'&&latest)drawHistory(latest.history||[]);
 setTimeout(()=>{renderScene();if(latest)drawHistory(latest.history||[])},50);
}
function showSettingsTab(id,btn){
 document.querySelectorAll('#view-settings .tabPane').forEach(x=>x.classList.remove('active'));
 document.querySelectorAll('#view-settings .tabBtn').forEach(x=>x.classList.remove('active'));
 $('settings-'+id).classList.add('active');btn.classList.add('active');
 if(id==='fc')loadFcParams();if(id==='geometry')loadGeometry();
}
function fmt(v,d=1){return v==null||!Number.isFinite(Number(v))?'—':Number(v).toFixed(d)}
function clamp(v,a,b){return Math.max(a,Math.min(b,v))}
function setActiveMode(mode){
 ['mStab','mPos'].forEach(id=>{if($(id))$(id).classList.remove('blue')});
 if(mode==='Stabilize'&&$('mStab'))$('mStab').classList.add('blue');
 if(mode==='PosHold'&&$('mPos'))$('mPos').classList.add('blue');
}
async function loadConfig(){
 let j=await api('/api/config'),c=j.runtime;
 $('focal').value=c.focal_scale;[$('r0').value,$('r1').value,$('r2').value,$('r3').value]=c.feature_roi;$('features').value=c.max_features;
 if($('sfocal')){$('sfocal').value=c.focal_scale;[$('sr0').value,$('sr1').value,$('sr2').value,$('sr3').value]=c.feature_roi;$('sfeatures').value=c.max_features;}
 window.visualizationMode=c.visualization_mode||'simple';refreshVisualizationMode();
 let g=j.geometry||{},gv=[];
 if(g.camera){gv.push(['FLOW_POS_X',g.camera.x*1000],['FLOW_POS_Y',g.camera.y*1000],['FLOW_POS_Z',g.camera.z*1000])}
 if(g.rangefinder){gv.push(['RNGFND1_POS_X',g.rangefinder.x*1000],['RNGFND1_POS_Y',g.rangefinder.y*1000],['RNGFND1_POS_Z',g.rangefinder.z*1000])}
 $('geomKv').innerHTML=gv.map(x=>'<span>'+x[0]+'</span><span>'+fmt(x[1],1)+'</span>').join('');
 let p=(j.fc_profile||{}).params||{},keys=['EK3_SRC1_POSXY','EK3_SRC1_VELXY','EK3_SRC1_POSZ','EK3_SRC1_YAW','FLOW_TYPE','FLOW_FXSCALER','FLOW_FYSCALER','EK3_FLOW_DELAY'];
 $('profileKv').innerHTML=keys.map(k=>'<span>'+k+'</span><span>'+(p[k]??'—')+'</span>').join('');
}
async function saveConfig(){
 try{
  let body={focal_scale:+$('focal').value,feature_roi:[+$('r0').value,+$('r1').value,+$('r2').value,+$('r3').value],max_features:+$('features').value,local_gui:true};
  await api('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  $('saveMsg').textContent='Сохранено';
 }catch(e){$('saveMsg').textContent='Ошибка: '+e.message}
}
async function saveRuntimeSettings(){
 try{
  let body={focal_scale:+$('sfocal').value,feature_roi:[+$('sr0').value,+$('sr1').value,+$('sr2').value,+$('sr3').value],max_features:+$('sfeatures').value};
  let j=await api('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  $('runtimeSettingsMsg').textContent='Сохранено';
  $('focal').value=j.runtime.focal_scale;[$('r0').value,$('r1').value,$('r2').value,$('r3').value]=j.runtime.feature_roi;$('features').value=j.runtime.max_features;
 }catch(e){$('runtimeSettingsMsg').textContent='Ошибка: '+e.message}
}
async function loadFcParams(){
 if(!$('fcParamRows'))return;
 $('fcParamMsg').textContent='Чтение...';
 try{
  let j=await api('/api/fc/params'),p=(j.profile||{}).params||{},vals=j.values||{};
  $('fcParamRows').innerHTML=Object.keys(p).map(k=>'<tr><td>'+k+'</td><td>'+fmt(vals[k],6)+'</td><td><input data-fc-param="'+k+'" value="'+(vals[k]??p[k])+'"></td></tr>').join('');
  $('fcParamMsg').textContent='Прочитано из FC';
 }catch(e){$('fcParamMsg').textContent='Ошибка: '+e.message}
}
async function writeFcParams(){
 if(!confirm('Записать изменённые критические параметры в FC? FC должен быть DISARMED.'))return;
 let values={};document.querySelectorAll('[data-fc-param]').forEach(e=>values[e.dataset.fcParam]=+String(e.value).replace(',','.'));
 $('fcParamMsg').textContent='Запись и проверка...';
 try{await api('/api/fc/params',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({values})});$('fcParamMsg').textContent='Записано и подтверждено';await loadFcParams();await loadConfig()}
 catch(e){$('fcParamMsg').textContent='Ошибка: '+e.message;alert(e.message)}
}
async function loadGeometry(){
 if(!$('gx0'))return;$('geometryMsg').textContent='Чтение...';
 try{
  let j=await api('/api/geometry'),v=j.values||{};
  $('gx0').value=fmt((v.FLOW_POS_X||0)*1000,1);$('gy0').value=fmt((v.FLOW_POS_Y||0)*1000,1);$('gz0').value=fmt((v.FLOW_POS_Z||0)*1000,1);
  $('grx').value=fmt((v.RNGFND1_POS_X||0)*1000,1);$('gry').value=fmt((v.RNGFND1_POS_Y||0)*1000,1);$('grz').value=fmt((v.RNGFND1_POS_Z||0)*1000,1);
  $('geometryMsg').textContent='Прочитано из FC';
 }catch(e){$('geometryMsg').textContent='Ошибка: '+e.message}
}
async function writeGeometry(){
 if(!confirm('Записать геометрию в FC и синхронизировать локальный config?'))return;
 let values={
  FLOW_POS_X:+String($('gx0').value).replace(',','.')/1000,FLOW_POS_Y:+String($('gy0').value).replace(',','.')/1000,FLOW_POS_Z:+String($('gz0').value).replace(',','.')/1000,
  RNGFND1_POS_X:+String($('grx').value).replace(',','.')/1000,RNGFND1_POS_Y:+String($('gry').value).replace(',','.')/1000,RNGFND1_POS_Z:+String($('grz').value).replace(',','.')/1000};
 $('geometryMsg').textContent='Запись и проверка...';
 try{await api('/api/geometry',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({values})});$('geometryMsg').textContent='Геометрия записана и синхронизирована';await loadGeometry();await loadConfig()}
 catch(e){$('geometryMsg').textContent='Ошибка: '+e.message;alert(e.message)}
}
let runRecording=false;
async function refreshRunRecordStatus(){
 try{
  let j=await api('/api/run-record/status');runRecording=!!j.recording;
  let b=$('runRecordBtn');if(b){b.textContent=runRecording?'■ ОСТАНОВКА ПРОГОНА':'● ЗАПИСЬ ПРОГОНА';b.classList.toggle('red',runRecording)}
  if(j.pending_name && !$('runNameModal').style.display.includes('flex'))openRunNameModal();
 }catch(e){}
}
async function toggleRunRecord(){
 try{
  if(!runRecording){
   await api('/api/run-record/start',{method:'POST'});runRecording=true;
  }else{
   await api('/api/run-record/stop',{method:'POST'});runRecording=false;openRunNameModal();
  }
  await refreshRunRecordStatus();
 }catch(e){alert(e.message)}
}
function openRunNameModal(){
 $('runNameModal').style.display='flex';$('runNameInput').value='';$('runNameMsg').textContent='';
 setTimeout(()=>$('runNameInput').focus(),50);
}
async function saveRunName(){
 let name=$('runNameInput').value.trim();if(!name){$('runNameMsg').textContent='Введите название прогона';return}
 try{
  await api('/api/run-record/finalize',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name})});
  $('runNameModal').style.display='none';await refreshJournal();
 }catch(e){$('runNameMsg').textContent=e.message}
}
async function openLogsModal(){
 $('logsModal').style.display='flex';$('logsList').innerHTML='Загрузка...';
 try{
  let j=await api('/api/run-logs'),logs=j.logs||[];
  $('logsList').innerHTML=logs.length?logs.map((x,i)=>'<label style="display:grid;grid-template-columns:24px 1fr auto;gap:8px;padding:8px;border-bottom:1px solid #173047"><input type="checkbox" class="runLogCheck" value="'+escapeHtml(x.name)+'"><span>'+escapeHtml(x.name)+'</span><span style="color:#86a7bf">'+(x.size/1024).toFixed(1)+' КБ</span></label>').join(''):'Логов пока нет';
 }catch(e){$('logsList').textContent='Ошибка: '+e.message}
}
function closeLogsModal(){$('logsModal').style.display='none'}
function downloadSelectedLogs(){
 let names=[...document.querySelectorAll('.runLogCheck:checked')].map(x=>x.value);
 if(!names.length){alert('Выберите хотя бы один лог');return}
 location.href='/api/run-logs/download?names='+encodeURIComponent(names.join('\n'));
}
async function refreshJournal(){
 if(!$('journalEvents'))return;
 try{let j=await api('/api/journal');$('journalEvents').innerHTML=(j.events||[]).slice().reverse().map(e=>'<div class="eventRow"><span class="ts">'+e.ts+'</span><span class="'+e.level+'">'+e.level+'</span><span>'+escapeHtml(e.text)+'</span></div>').join('')}
 catch(e){}
}
function escapeHtml(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
async function refreshMessages(){
 try{let j=await api('/api/messages'),ev=j.events||[];$('log').innerHTML=ev.slice(-120).map(e=>'['+e.ts+'] [S'+e.severity+'] '+escapeHtml(e.text)).join('\n');$('log').scrollTop=$('log').scrollHeight}catch(e){}
}
function refreshVisualizationMode(){
 let m=window.visualizationMode||'simple';
 if(m!=='simple'&&m!=='light')m='simple';
 ['Simple','Light'].forEach(x=>{let e=$('vm'+x);if(e)e.classList.remove('active')});
 let id=m==='light'?'vmLight':'vmSimple';if($(id))$(id).classList.add('active');
 if($('glCanvas')&&$('lightCanvas')){
  $('glCanvas').style.display=m==='simple'?'block':'none';
  $('lightCanvas').style.display=m==='light'?'block':'none';
 }
 if($('visualModeMsg'))$('visualModeMsg').textContent=m==='light'?'Лёгкий 2D режим: WebGL отключён.':'Упрощённый WebGL режим.';
 if(latest){if(m==='light')drawLightScene(latest);else renderScene()}
}
async function setVisualizationMode(mode){
 try{let j=await api('/api/system/visualization',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode})});window.visualizationMode=j.runtime.visualization_mode;refreshVisualizationMode()}
 catch(e){alert(e.message)}
}
function drawLightScene(t){
 let c=$('lightCanvas');if(!c)return;let ctx=c.getContext('2d');let d=devicePixelRatio,w=c.width=c.clientWidth*d,h=c.height=c.clientHeight*d;ctx.scale(d,d);w=c.clientWidth;h=c.clientHeight;
 ctx.fillStyle='#07121c';ctx.fillRect(0,0,w,h);let cx=w/2,cy=h/2,scale=Math.min(w,h)/2.5;
 ctx.strokeStyle='#153d58';for(let i=-5;i<=5;i++){let q=i*.2*scale;ctx.beginPath();ctx.moveTo(cx+q,20);ctx.lineTo(cx+q,h-20);ctx.stroke();ctx.beginPath();ctx.moveTo(20,cy+q);ctx.lineTo(w-20,cy+q);ctx.stroke()}
 let tr=t.trail||[];ctx.strokeStyle='#1eaaff';ctx.lineWidth=2;ctx.beginPath();tr.forEach((p,i)=>{let x=cx+(p.y_mm/1000)*scale,y=cy+(p.x_mm/1000)*scale;if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y)});ctx.stroke();
 let x=cx+((t.y_mm||0)/1000)*scale,y=cy+((t.x_mm||0)/1000)*scale;ctx.fillStyle='#17d878';ctx.beginPath();ctx.arc(x,y,8,0,Math.PI*2);ctx.fill();ctx.fillStyle='#9fb7c9';ctx.fillText('N ↑   E →',16,22);
}
async function start(){
 let box=$('runtimeError');box.style.display='none';box.textContent='';
 try{
  await api('/api/start',{method:'POST'});
  setTimeout(refreshRuntimeStatus,300);
 }catch(e){
  box.style.display='block';box.textContent=e.message;alert('Не удалось запустить flight runtime. Причина показана под кнопкой запуска.');
 }
}
async function stop(){try{await api('/api/stop',{method:'POST'});setTimeout(refreshRuntimeStatus,150);}catch(e){alert(e.message)}}
async function restartEstimator(){
 const b=$('restartEstimatorBtn');if(!b)return;
 if(!confirm('Перезапустить оценку и принять текущую точку за новый локальный ноль?'))return;
 const old=b.textContent;b.disabled=true;b.textContent='↻ ПЕРЕЗАПУСК…';
 try{
  const j=await api('/api/restart-fast',{method:'POST'});
  wsHistory=[];wsTrail=[];wsT0=null;resetMotionCompare();
  b.textContent=j.ready?'✓ ОЦЕНКА ГОТОВА · НОВЫЙ НОЛЬ':'⚠ ПЕРЕЗАПУЩЕНО · ОЖИДАНИЕ ОЦЕНКИ';
  setTimeout(()=>{b.textContent=old;b.disabled=false;refreshRuntimeStatus()},1400);
 }catch(e){
  b.textContent=old;b.disabled=false;alert(e.message);
 }
}
async function zero(){try{
 await api('/api/zero',{method:'POST'});
 wsHistory=[];wsTrail=[];wsT0=null;resetMotionCompare();
}catch(e){alert(e.message)}}
async function armFc(){if(!confirm('ARM: разрешить запуск моторов?'))return;try{showFc(await api('/api/fc/arm',{method:'POST'}))}catch(e){alert(e.message)}}
async function disarmFc(){if(!confirm('DISARM: отключить моторы?'))return;try{showFc(await api('/api/fc/disarm',{method:'POST'}))}catch(e){alert(e.message)}}
async function setMode(id,name){if(!confirm('Переключить режим на '+name+'?'))return;try{showFc(await api('/api/fc/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:id})}))}catch(e){alert(e.message)}}
async function takeoffFc(){
 const alt=Number(String($('takeoffAlt').value).replace(',','.'));
 if(!Number.isFinite(alt)||alt<0.10||alt>10.0){alert('Высота TAKEOFF должна быть 0.10..10.0 м');return}
 if(!confirm('TAKEOFF: перейти в Guided и подняться на +'+alt.toFixed(2)+' м относительно точки старта?'))return;
 try{showFc(await api('/api/fc/takeoff',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({alt_m:alt})}))}catch(e){alert(e.message)}
}
async function landFc(){
 if(!confirm('LAND: перевести FC в режим посадки?'))return;
 try{showFc(await api('/api/fc/land',{method:'POST'}))}catch(e){alert(e.message)}
}
function showFc(j){
 fcLatest=j;$('linkDot').classList.remove('baddot');$('linkText').textContent='OK';$('fcDotBig').classList.remove('baddot');
 $('fcState').textContent=j.armed?'ARMED':'DISARMED';$('fcState').style.color=j.armed?'#ff5967':'#e9f3fb';$('fcMode').textContent=j.mode;$('arm').textContent=j.armed?'ARMED':'DISARMED';setActiveMode(j.mode);
}
async function refreshFc(){try{showFc(await api('/api/fc'))}catch(e){$('linkDot').classList.add('baddot');$('linkText').textContent='НЕТ';$('fcDotBig').classList.add('baddot');$('fcState').textContent='НЕТ СВЯЗИ';$('fcMode').textContent='—'}}

function resetMotionCompare(){
 motionTrails={cam:[],imu:[],fused:[],ekf:[]};
 drawMotionCompare();
}
function motionPoint(t,key){
 if(key==='cam' && t.raw_of_n_mm!=null && t.raw_of_e_mm!=null)return {n:Number(t.raw_of_n_mm),e:Number(t.raw_of_e_mm)};
 if(key==='ekf')return {n:Number(t.x_mm||0),e:Number(t.y_mm||0)};
 if(key==='imu' && t.imu_dr_n_mm!=null && t.imu_dr_e_mm!=null)return {n:Number(t.imu_dr_n_mm),e:Number(t.imu_dr_e_mm)};
 if(key==='fused' && t.fused_v1_n_mm!=null && t.fused_v1_e_mm!=null)return {n:Number(t.fused_v1_n_mm),e:Number(t.fused_v1_e_mm)};
 return null;
}
function pushMotionCompare(t){
 for(const k of ['cam','imu','fused','ekf']){
   const p=motionPoint(t,k);if(!p||!Number.isFinite(p.n)||!Number.isFinite(p.e))continue;
   motionTrails[k].push(p);if(motionTrails[k].length>MOTION_MAX_POINTS)motionTrails[k].splice(0,motionTrails[k].length-MOTION_MAX_POINTS);
 }
 drawMotionCompare();
}
function drawMotionCompare(){
 const c=$('motionCompare');if(!c)return;const d=devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;
 c.width=Math.max(2,Math.floor(w*d));c.height=Math.max(2,Math.floor(h*d));const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);
 x.fillStyle='#06111a';x.fillRect(0,0,w,h);
 let maxAbs=100;
 for(const tr of Object.values(motionTrails))for(const p of tr)maxAbs=Math.max(maxAbs,Math.abs(p.n),Math.abs(p.e));
 const half=Math.max(100,Math.ceil(maxAbs/100)*100),pad=34,scale=Math.min((w-2*pad)/(2*half),(h-2*pad)/(2*half)),cx=w/2,cy=h/2;
 x.font='11px system-ui';x.textAlign='left';x.textBaseline='middle';
 const step=50;
 x.lineWidth=1;
 for(let q=-half;q<=half+1e-6;q+=step){
   x.strokeStyle='#153247';x.beginPath();x.moveTo(cx+q*scale,pad);x.lineTo(cx+q*scale,h-pad);x.stroke();
   x.beginPath();x.moveTo(pad,cy-q*scale);x.lineTo(w-pad,cy-q*scale);x.stroke();
 }
 x.strokeStyle='#55768e';x.lineWidth=1.5;x.beginPath();x.moveTo(pad,cy);x.lineTo(w-pad,cy);x.stroke();x.beginPath();x.moveTo(cx,pad);x.lineTo(cx,h-pad);x.stroke();
 x.fillStyle='#8da9bd';x.fillText('N',cx+6,pad+7);x.fillText('E',w-pad-12,cy-10);x.fillText('±'+half+' мм',8,14);
 const cfg={ekf:['#0bd777','FC EKF']};
 for(const [k,[col]] of Object.entries(cfg)){
   const tr=motionTrails[k];if(!tr.length)continue;x.strokeStyle=col;x.lineWidth=2.4;x.beginPath();
   tr.forEach((p,i)=>{const px=cx+p.e*scale,py=cy-p.n*scale;i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke();
   const p=tr[tr.length-1],px=cx+p.e*scale,py=cy-p.n*scale;x.fillStyle=col;x.beginPath();x.arc(px,py,5,0,Math.PI*2);x.fill();
 }
 function label(id,k){const tr=motionTrails[k],el=$(id);if(!el)return;if(!tr.length){el.textContent='—';return}const p=tr[tr.length-1];el.textContent='N '+fmt(p.n,1)+' · E '+fmt(p.e,1)+' · |XY| '+fmt(Math.hypot(p.n,p.e),1)+' мм'}
 label('cmpEkf','ekf');
}
function updateHud(t){
 latest=t;window.latest=t;$('runState').textContent=t.running?'Работает':'Остановлен';
 if(!t.available){
   $('footerRuntime').textContent=t.running?'запускается…':'остановлен';
   $('mx').textContent='—';$('my').textContent='—';$('mz').textContent='—';$('mr').textContent='—';$('mq').textContent='—';
   ['imuX','imuY','imuZ','imuStationary','imuAcc','imuVel','imuDt','imuAmag','imuGmag','imuStatSamples','imuAccRejects','imuGyroRejects','imuCamStat','camvcX','camvcY','camvcZ','camvcVel','camvcState','camX','camY','camZ','fusedX','fusedY','fusedZ','fusedState'].forEach(id=>{if($(id))$(id).textContent='—'});
   $('frame').textContent='—';$('inl').textContent='—';$('ekf').textContent='—';
   ['ekfNE','ekfDrift','ekfVel','rawNE','rawDrift','rawVel'].forEach(id=>{if($(id))$(id).textContent='—'});
   $('sceneXYZ').textContent=t.running?'Ожидание WebSocket телеметрии…':'Runtime остановлен — live данные отсутствуют';
   let box=$('runtimeError');
   if(t.runtime_exit&&t.runtime_exit.log_tail){
     box.style.display='block';
     box.textContent='Runtime завершён (code '+t.runtime_exit.returncode+')\n'+t.runtime_exit.log_tail;
   }
   clearCharts();
   return;
 }
 $('runState').textContent=t.running?'Работает':'Остановлен';$('footerRuntime').textContent=t.running?'работает':'остановлен';$('footerRuntime').style.color=t.running?'#15d876':'#8aa5b8';
 $('mx').textContent=fmt(t.x_mm,0)+' мм';$('my').textContent=fmt(t.y_mm,0)+' мм';$('mz').textContent=fmt(t.z_mm,0)+' мм';$('mr').textContent=t.range_m==null?'—':fmt(t.range_m*1000,0)+' мм';$('mq').textContent=t.raw_of_valid?'VALID':'INVALID';
 if($('imuX'))$('imuX').textContent=fmt(t.imu_dr_n_mm,0)+' мм';
 if($('imuY'))$('imuY').textContent=fmt(t.imu_dr_e_mm,0)+' мм';
 if($('imuZ'))$('imuZ').textContent=fmt(t.imu_dr_d_mm,0)+' мм';
 if($('imuStationary'))$('imuStationary').textContent=t.imu_dr_stationary?'ДА':'НЕТ';
 if($('imuAcc'))$('imuAcc').textContent=fmt(t.imu_dr_acc_n,4)+' / '+fmt(t.imu_dr_acc_e,4)+' / '+fmt(t.imu_dr_acc_d,4)+' м/с²';
 if($('imuVel'))$('imuVel').textContent=fmt(t.imu_dr_vn,3)+' / '+fmt(t.imu_dr_ve,3)+' / '+fmt(t.imu_dr_vd,3)+' м/с';
 if($('imuDt'))$('imuDt').textContent=fmt((t.imu_dr_dt||0)*1000,2)+' мс';
 if($('imuAmag'))$('imuAmag').textContent=fmt(t.imu_dr_amag,4)+' · '+(t.imu_dr_acc_ok?'OK':'REJECT');
 if($('imuGmag'))$('imuGmag').textContent=fmt(t.imu_dr_gmag,5)+' · '+(t.imu_dr_gyro_ok?'OK':'REJECT');
 if($('imuStatSamples'))$('imuStatSamples').textContent=t.imu_dr_stationary_samples??'—';
 if($('imuAccRejects'))$('imuAccRejects').textContent=t.imu_dr_acc_rejects??'—';
 if($('imuGyroRejects'))$('imuGyroRejects').textContent=t.imu_dr_gyro_rejects??'—';
 if($('imuCamStat'))$('imuCamStat').textContent=t.imu_cam_stationary?'ДА':'НЕТ';
 if($('camvcX'))$('camvcX').textContent=fmt(t.imu_camvc_n_mm,0)+' мм';
 if($('camvcY'))$('camvcY').textContent=fmt(t.imu_camvc_e_mm,0)+' мм';
 if($('camvcZ'))$('camvcZ').textContent='—';
 if($('camvcVel'))$('camvcVel').textContent=fmt(t.imu_camvc_vn,3)+' / '+fmt(t.imu_camvc_ve,3);
 if($('camvcState'))$('camvcState').textContent=t.imu_camvc_active?'ACTIVE':'INACTIVE';
 if($('camX'))$('camX').textContent=fmt(t.x_mm,0)+' мм';
 if($('camY'))$('camY').textContent=fmt(t.y_mm,0)+' мм';
 if($('camZ'))$('camZ').textContent=fmt(t.z_mm,0)+' мм';
 if($('fusedX'))$('fusedX').textContent=fmt(t.fused_v1_n_mm,0)+' мм';
 if($('fusedY'))$('fusedY').textContent=fmt(t.fused_v1_e_mm,0)+' мм';
 if($('fusedZ'))$('fusedZ').textContent='—';
 if($('fusedState'))$('fusedState').textContent=t.fused_v1_stationary?'STATIONARY':'MOTION';
 $('roll').textContent=fmt(t.roll_deg,1)+'°';$('pitch').textContent=fmt(t.pitch_deg,1)+'°';$('yaw').textContent=fmt(t.yaw_deg,1)+'°';
 $('inl').textContent=(t.inliers??'—')+'/'+(t.tracked??'—');$('frame').textContent=t.frame??'—';$('ekf').textContent=t.ekf_valid?'VALID':'NO DATA';
 if($('ekfNE'))$('ekfNE').textContent=fmt(t.x_mm,1)+' / '+fmt(t.y_mm,1)+' мм';
 if($('ekfDrift'))$('ekfDrift').textContent=fmt(t.ekf_drift_mm,1)+' мм';
 if($('ekfVel'))$('ekfVel').textContent=fmt((t.vx||0)*1000,1)+' / '+fmt((t.vy||0)*1000,1)+' мм/с';
 if($('rawNE'))$('rawNE').textContent=t.raw_of_n_mm==null?'—':fmt(t.raw_of_n_mm,1)+' / '+fmt(t.raw_of_e_mm,1)+' мм';
 if($('rawDrift'))$('rawDrift').textContent=t.raw_of_drift_mm==null?'—':fmt(t.raw_of_drift_mm,1)+' мм';
 if($('rawVel'))$('rawVel').textContent=t.raw_of_vn==null?'—':fmt(t.raw_of_vn*1000,1)+' / '+fmt(t.raw_of_ve*1000,1)+' мм/с';
 $('sceneXYZ').textContent='X '+fmt((t.x_mm||0)/1000,3)+' · Y '+fmt((t.y_mm||0)/1000,3)+' · Z '+fmt((t.z_mm||0)/1000,3)+' m';
 $('rollNeedle').style.left=(50+clamp(t.roll_deg||0,-45,45)/45*50)+'%';
 $('pitchNeedle').style.left=(50+clamp(t.pitch_deg||0,-45,45)/45*50)+'%';
 let y=((t.yaw_deg||0)+180)%360-180;$('yawNeedle').style.left=(50+y/180*50)+'%';
 if($('tmx')){$('tmx').textContent=fmt(t.x_mm,0)+' мм';$('tmy').textContent=fmt(t.y_mm,0)+' мм';$('tmz').textContent=fmt(t.z_mm,0)+' мм';$('tmr').textContent=t.range_m==null?'—':fmt(t.range_m*1000,0)+' мм';$('tmq').textContent=t.quality??'—';$('troll').textContent=fmt(t.roll_deg,1)+'°';$('tpitch').textContent=fmt(t.pitch_deg,1)+'°';$('tyaw').textContent=fmt(t.yaw_deg,1)+'°';$('tinl').textContent=(t.inliers??'—')+'/'+(t.tracked??'—');$('tekf').textContent=t.ekf_valid?'VALID':'NO DATA'}
 drawCompass(t.yaw_deg||0);
 const now=performance.now();
 if(now-lastChartPaint>500){drawHistory(t.history||[]);lastChartPaint=now;}
 let vm=window.visualizationMode||'simple';if(vm==='light')drawLightScene(t);else renderScene()
}

function drawCompass(deg){
 let c=$('compass'),ctx=c.getContext('2d'),w=c.width,h=c.height,cx=w/2,cy=h/2,R=82;ctx.clearRect(0,0,w,h);
 ctx.strokeStyle='#294a62';ctx.lineWidth=3;ctx.beginPath();ctx.arc(cx,cy,R,0,Math.PI*2);ctx.stroke();
 ctx.font='12px system-ui';ctx.fillStyle='#cad9e4';ctx.textAlign='center';ctx.textBaseline='middle';
 [['N',0],['E',90],['S',180],['W',270]].forEach(([q,d])=>{let a=(d-90)*Math.PI/180;ctx.fillText(q,cx+Math.cos(a)*(R-13),cy+Math.sin(a)*(R-13))});
 for(let d=0;d<360;d+=10){let a=(d-90)*Math.PI/180,r1=R-4,r2=d%30===0?R-13:R-9;ctx.strokeStyle='#557188';ctx.lineWidth=d%30===0?2:1;ctx.beginPath();ctx.moveTo(cx+Math.cos(a)*r1,cy+Math.sin(a)*r1);ctx.lineTo(cx+Math.cos(a)*r2,cy+Math.sin(a)*r2);ctx.stroke()}
 let a=(deg-90)*Math.PI/180;ctx.fillStyle='#1ca8ff';ctx.beginPath();ctx.moveTo(cx+Math.cos(a)*(R-24),cy+Math.sin(a)*(R-24));ctx.lineTo(cx+Math.cos(a+2.55)*20,cy+Math.sin(a+2.55)*20);ctx.lineTo(cx+Math.cos(a-2.55)*20,cy+Math.sin(a-2.55)*20);ctx.closePath();ctx.fill();
 ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(cx,cy,5,0,Math.PI*2);ctx.fill();ctx.font='bold 22px system-ui';ctx.fillText(fmt(deg,0)+'°',cx,cy+42);
}
function clearCharts(){
 ['xyzChart','speedChart','rangeChart','tXyzChart','tSpeedChart','tRangeChart'].forEach(id=>{
   let c=$(id);if(!c)return;
   let ctx=c.getContext('2d');
   let w=c.width=Math.max(2,c.clientWidth*devicePixelRatio),h=c.height=Math.max(2,c.clientHeight*devicePixelRatio);
   ctx.setTransform(1,0,0,1,0,0);ctx.clearRect(0,0,w,h);
 });
}
function chartBase(c,ctx){
 let w=c.width=c.clientWidth*devicePixelRatio,h=c.height=c.clientHeight*devicePixelRatio;ctx.scale(devicePixelRatio,devicePixelRatio);w=c.clientWidth;h=c.clientHeight;
 ctx.fillStyle='#08131d';ctx.fillRect(0,0,w,h);ctx.strokeStyle='#183246';ctx.lineWidth=1;
 for(let i=1;i<5;i++){let y=i*h/5;ctx.beginPath();ctx.moveTo(32,y);ctx.lineTo(w-8,y);ctx.stroke()}
 for(let i=1;i<6;i++){let x=32+i*(w-40)/6;ctx.beginPath();ctx.moveTo(x,10);ctx.lineTo(x,h-22);ctx.stroke()}
 return [w,h];
}
function plotSeries(ctx,data,key,min,max,color,w,h){
 let vals=data.map(d=>d[key]).filter(v=>v!=null&&Number.isFinite(v));if(!vals.length)return;
 if(min==null){min=Math.min(...vals);max=Math.max(...vals);if(Math.abs(max-min)<1e-6){min-=1;max+=1}else{let p=(max-min)*.15;min-=p;max+=p}}
 ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();let started=false;
 data.forEach((d,i)=>{let v=d[key];if(v==null||!Number.isFinite(v))return;let x=32+i*(w-44)/Math.max(1,data.length-1),y=10+(max-v)/(max-min)*(h-34);if(!started){ctx.moveTo(x,y);started=true}else ctx.lineTo(x,y)});ctx.stroke();
}
function drawHistory(h){
 let c=$('xyzChart'),ctx=c.getContext('2d'),[w,hh]=chartBase(c,ctx);let vals=[];h.forEach(d=>['x','y','z'].forEach(k=>{if(d[k]!=null)vals.push(d[k])}));let m=Math.max(.05,...vals.map(Math.abs));plotSeries(ctx,h,'x',-m,m,'#ff4352',w,hh);plotSeries(ctx,h,'y',-m,m,'#16d878',w,hh);plotSeries(ctx,h,'z',-m,m,'#218cff',w,hh);
 c=$('speedChart');ctx=c.getContext('2d');[w,hh]=chartBase(c,ctx);plotSeries(ctx,h,'speed',0,Math.max(.2,...h.map(d=>d.speed||0))*1.15,'#ffd11f',w,hh);
 c=$('rangeChart');ctx=c.getContext('2d');[w,hh]=chartBase(c,ctx);let rv=h.map(d=>d.range).filter(v=>v!=null),rmax=Math.max(.5,...rv)*1.2;plotSeries(ctx,h,'range',0,rmax,'#c98cff',w,hh);
 if($('view-telemetry')?.classList.contains('activeView')&&$('tXyzChart')){c=$('tXyzChart');ctx=c.getContext('2d');[w,hh]=chartBase(c,ctx);plotSeries(ctx,h,'x',-m,m,'#ff4352',w,hh);plotSeries(ctx,h,'y',-m,m,'#16d878',w,hh);plotSeries(ctx,h,'z',-m,m,'#218cff',w,hh)}
 if($('view-telemetry')?.classList.contains('activeView')&&$('tSpeedChart')){c=$('tSpeedChart');ctx=c.getContext('2d');[w,hh]=chartBase(c,ctx);plotSeries(ctx,h,'speed',0,Math.max(.2,...h.map(d=>d.speed||0))*1.15,'#ffd11f',w,hh)}
 if($('view-telemetry')?.classList.contains('activeView')&&$('tRangeChart')){c=$('tRangeChart');ctx=c.getContext('2d');[w,hh]=chartBase(c,ctx);plotSeries(ctx,h,'range',0,rmax,'#c98cff',w,hh)}
}

let gl,prog,bufPos,bufCol,locMvp,locPos,locCol;
function m4mul(a,b){let o=new Float32Array(16);for(let c=0;c<4;c++)for(let r=0;r<4;r++){let v=0;for(let k=0;k<4;k++)v+=a[k*4+r]*b[c*4+k];o[c*4+r]=v}return o}
function perspective(fov,asp,n,f){let t=1/Math.tan(fov/2);return new Float32Array([t/asp,0,0,0,0,t,0,0,0,0,(f+n)/(n-f),-1,0,0,2*f*n/(n-f),0])}
function lookAt(e,t,u){let z=norm(sub(e,t)),x=norm(cross(u,z)),y=cross(z,x);return new Float32Array([x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-dot(x,e),-dot(y,e),-dot(z,e),1])}
const sub=(a,b)=>a.map((v,i)=>v-b[i]),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],norm=a=>{let l=Math.hypot(...a)||1;return a.map(v=>v/l)}
function initGL(){
 let c=$('glCanvas');gl=c.getContext('webgl',{antialias:true,alpha:false});if(!gl)return;
 let vs=gl.createShader(gl.VERTEX_SHADER);gl.shaderSource(vs,'attribute vec3 p;attribute vec3 c;uniform mat4 m;varying vec3 v;void main(){gl_Position=m*vec4(p,1.0);v=c;gl_PointSize=8.0;}');gl.compileShader(vs);
 let fs=gl.createShader(gl.FRAGMENT_SHADER);gl.shaderSource(fs,'precision mediump float;varying vec3 v;void main(){gl_FragColor=vec4(v,1.0);}');gl.compileShader(fs);
 prog=gl.createProgram();gl.attachShader(prog,vs);gl.attachShader(prog,fs);gl.linkProgram(prog);gl.useProgram(prog);
 locPos=gl.getAttribLocation(prog,'p');locCol=gl.getAttribLocation(prog,'c');locMvp=gl.getUniformLocation(prog,'m');bufPos=gl.createBuffer();bufCol=gl.createBuffer();
 c.onmousedown=e=>{drag=true;lastX=e.clientX;lastY=e.clientY};window.onmouseup=()=>drag=false;window.onmousemove=e=>{if(!drag)return;viewYaw+=(e.clientX-lastX)*.008;viewPitch=clamp(viewPitch+(e.clientY-lastY)*.008,.1,1.45);lastX=e.clientX;lastY=e.clientY;renderScene()};
 c.onwheel=e=>{e.preventDefault();viewDist=clamp(viewDist+e.deltaY*.005,0.8,11);renderScene()};
}
function addLine(P,C,a,b,col){P.push(...a,...b);C.push(...col,...col)}
function addThickLine(P,C,a,b,col,r=.014){
 let dx=b[0]-a[0],dy=b[1]-a[1],dz=b[2]-a[2],L=Math.hypot(dx,dy,dz)||1;
 let ux=dx/L,uy=dy/L,uz=dz/L,rx=Math.abs(uz)<.9?0:1,ry=0,rz=Math.abs(uz)<.9?1:0;
 let vx=uy*rz-uz*ry,vy=uz*rx-ux*rz,vz=ux*ry-uy*rx,V=Math.hypot(vx,vy,vz)||1;vx/=V;vy/=V;vz/=V;
 let wx=uy*vz-uz*vy,wy=uz*vx-ux*vz,wz=ux*vy-uy*vx,ring=[];
 for(let i=0;i<8;i++){let q=i*Math.PI/4;ring.push([r*(Math.cos(q)*vx+Math.sin(q)*wx),r*(Math.cos(q)*vy+Math.sin(q)*wy),r*(Math.cos(q)*vz+Math.sin(q)*wz)])}
 for(let i=0;i<8;i++){let o=ring[i],n=ring[(i+1)%8];addLine(P,C,[a[0]+o[0],a[1]+o[1],a[2]+o[2]],[b[0]+o[0],b[1]+o[1],b[2]+o[2]],col);addLine(P,C,[a[0]+o[0],a[1]+o[1],a[2]+o[2]],[a[0]+n[0],a[1]+n[1],a[2]+n[2]],col);addLine(P,C,[b[0]+o[0],b[1]+o[1],b[2]+o[2]],[b[0]+n[0],b[1]+n[1],b[2]+n[2]],col)}
}
function addCircle(P,C,center,r,col,plane='xy'){let n=28;for(let i=0;i<n;i++){let a=i/n*Math.PI*2,b=(i+1)/n*Math.PI*2,A=[...center],B=[...center];if(plane==='xy'){A[0]+=Math.cos(a)*r;A[1]+=Math.sin(a)*r;B[0]+=Math.cos(b)*r;B[1]+=Math.sin(b)*r}else{A[0]+=Math.cos(a)*r;A[2]+=Math.sin(a)*r;B[0]+=Math.cos(b)*r;B[2]+=Math.sin(b)*r}addLine(P,C,A,B,col)}}
function rotLocal(p,r,pit,y){let cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(pit),sp=Math.sin(pit),cy=Math.cos(y),sy=Math.sin(y);let [x,Y,z]=p;let y1=cr*Y-sr*z,z1=sr*Y+cr*z,x1=x;let x2=cp*x1+sp*z1,y2=y1,z2=-sp*x1+cp*z1;return [cy*x2-sy*y2,sy*x2+cy*y2,z2]}
function renderScene(){
 if(!gl)return;let c=$('glCanvas'),dpr=devicePixelRatio,w=Math.floor(c.clientWidth*dpr),h=Math.floor(c.clientHeight*dpr);if(c.width!==w||c.height!==h){c.width=w;c.height=h}gl.viewport(0,0,w,h);gl.clearColor(.025,.065,.095,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.enable(gl.DEPTH_TEST);
 let P=[],C=[];
 if($('showGrid').checked){const step=.05,extent=2.5;for(let q=-extent;q<=extent+1e-9;q+=step){addLine(P,C,[-extent,q,0],[extent,q,0],[.08,.23,.34]);addLine(P,C,[q,-extent,0],[q,extent,0],[.08,.23,.34])}}
 if($('showAxes').checked){addThickLine(P,C,[0,0,0],[1.15,0,0],[1,.15,.15],.010);addThickLine(P,C,[0,0,0],[0,1.15,0],[.1,1,.25],.010);addThickLine(P,C,[0,0,0],[0,0,1.15],[.1,.45,1],.010)}
 addCircle(P,C,[0,0,.01],.08,[.1,1,.35]);
 if(latest&&$('showTrail').checked&&(latest.trail||[]).length>1){let tr=latest.trail;for(let i=1;i<tr.length;i++){let a=tr[i-1],b=tr[i];addThickLine(P,C,[a.x_mm/1000,a.y_mm/1000,-(a.z_mm||0)/1000],[b.x_mm/1000,b.y_mm/1000,-(b.z_mm||0)/1000],[.05,.75,1],.012)}}
 let pos=latest?[(latest.x_mm||0)/1000,(latest.y_mm||0)/1000,-(latest.z_mm||0)/1000]:[0,0,.2],rr=(latest?.roll_deg||0)*Math.PI/180,pp=(latest?.pitch_deg||0)*Math.PI/180,yy=(latest?.yaw_deg||0)*Math.PI/180;
 function wp(v){let q=rotLocal(v,rr,pp,yy);return[q[0]+pos[0],q[1]+pos[1],q[2]+pos[2]]}
 {
   let arm=.16;addLine(P,C,wp([arm,arm,0]),wp([-arm,-arm,0]),[.7,.78,.84]);addLine(P,C,wp([arm,-arm,0]),wp([-arm,arm,0]),[.7,.78,.84]);
   [[arm,arm],[-arm,-arm],[arm,-arm],[-arm,arm]].forEach((xy,i)=>{let n=30;for(let k=0;k<n;k++){let a=k/n*Math.PI*2,b=(k+1)/n*Math.PI*2,A=wp([xy[0]+Math.cos(a)*.085,xy[1]+Math.sin(a)*.085,.015]),B=wp([xy[0]+Math.cos(b)*.085,xy[1]+Math.sin(b)*.085,.015]);addLine(P,C,A,B,i<2?[.1,.9,.55]:[.25,.55,1])}});
   let body=[[-.06,-.04,-.025],[.06,-.04,-.025],[.06,.04,-.025],[-.06,.04,-.025],[-.06,-.04,.035],[.06,-.04,.035],[.06,.04,.035],[-.06,.04,.035]],edges=[[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];edges.forEach(e=>addLine(P,C,wp(body[e[0]]),wp(body[e[1]]),[1,.45,.08]));addLine(P,C,wp([.05,0,.01]),wp([.24,0,.01]),[1,.1,.1]);
 }
 let a=viewYaw,p=viewPitch;if(viewMode==='top'){a=0;p=.05}else if(viewMode==='front'){a=Math.PI/2;p=.4}else if(viewMode==='side'){a=0;p=.4}
 let eye=[Math.cos(a)*Math.cos(p)*viewDist,Math.sin(a)*Math.cos(p)*viewDist,Math.sin(p)*viewDist],target=$('followCam').checked?pos:[0,0,.25],V=lookAt(eye,target,[0,0,1]),Pr=perspective(.8,w/h,.05,40),M=m4mul(Pr,V);gl.uniformMatrix4fv(locMvp,false,M);
 gl.bindBuffer(gl.ARRAY_BUFFER,bufPos);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(P),gl.DYNAMIC_DRAW);gl.enableVertexAttribArray(locPos);gl.vertexAttribPointer(locPos,3,gl.FLOAT,false,0,0);
 gl.bindBuffer(gl.ARRAY_BUFFER,bufCol);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(C),gl.DYNAMIC_DRAW);gl.enableVertexAttribArray(locCol);gl.vertexAttribPointer(locCol,3,gl.FLOAT,false,0,0);gl.drawArrays(gl.LINES,0,P.length/3);
}
function setView(v,el){viewMode=v;document.querySelectorAll('.viewItem').forEach(x=>x.classList.remove('active'));el.classList.add('active');renderScene()}
function resetView(){viewMode='iso';viewYaw=.75;viewPitch=.65;viewDist=6.4;renderScene()}

function ingestWsTelemetry(t){
 if(t.type==='zero'){
   wsHistory=[];wsTrail=[];wsT0=null;resetMotionCompare();
   return;
 }
 if(t.type==='runtime'){
   if(!t.running){
     wsHistory=[];wsTrail=[];wsT0=null;
     updateHud({available:false,running:false,runtime_exit:null});
   }
   return;
 }
 if(t.type!=='telemetry')return;
 if(t.rc_zero_event){
   wsHistory=[];wsTrail=[];wsT0=null;resetMotionCompare();
 }
 const mono=Number(t.mono_ns||0);
 if(wsT0===null && mono>0)wsT0=mono;
 const vx=Number(t.vx||0),vy=Number(t.vy||0),vz=Number(t.vz||0);
 wsHistory.push({
   t:(mono>0&&wsT0!==null)?(mono-wsT0)/1e9:0,
   x:Number(t.x_mm||0)/1000,
   y:Number(t.y_mm||0)/1000,
   z:Number(t.z_mm||0)/1000,
   speed:Math.hypot(vx,vy,vz),
   range:t.range_m==null?null:Number(t.range_m)
 });
 wsTrail.push({x_mm:Number(t.x_mm||0),y_mm:Number(t.y_mm||0),z_mm:Number(t.z_mm||0)});
 if(wsHistory.length>WS_MAX_POINTS)wsHistory.splice(0,wsHistory.length-WS_MAX_POINTS);
 if(wsTrail.length>WS_MAX_POINTS)wsTrail.splice(0,wsTrail.length-WS_MAX_POINTS);
 t.history=wsHistory;
 t.trail=wsTrail;
 t.available=true;
 t.running=true;
 pushMotionCompare(t);
 updateHud(t);
}
function connectTelemetryWs(){
 if(telemetryWs && (telemetryWs.readyState===WebSocket.OPEN||telemetryWs.readyState===WebSocket.CONNECTING))return;
 const proto=location.protocol==='https:'?'wss':'ws';
 telemetryWs=new WebSocket(proto+'://'+location.host+'/ws/telemetry');
 telemetryWs.onopen=()=>{
   const box=$('runtimeError');
   if(box && box.textContent.startsWith('WebSocket')){box.style.display='none';box.textContent='';}
 };
 telemetryWs.onmessage=e=>{
   try{ingestWsTelemetry(JSON.parse(e.data))}catch(err){console.error('telemetry ws',err)}
 };
 telemetryWs.onerror=()=>{};
 telemetryWs.onclose=()=>{
   telemetryWs=null;
   clearTimeout(wsReconnectTimer);
   wsReconnectTimer=setTimeout(connectTelemetryWs,1000);
 };
}
async function refreshRuntimeStatus(){
 try{
   const st=await api('/api/status');
   $('runState').textContent=st.running?'Работает':'Остановлен';
   const timerBox=$('testTimerBox'),timerText=$('testTimerText'),timerBar=$('testTimerBar');
   if(st.running && st.test_duration_s && st.test_remaining_s!=null){
     const dur=Math.max(1,Number(st.test_duration_s)),rem=Math.max(0,Number(st.test_remaining_s));
     const sec=Math.ceil(rem),mm=Math.floor(sec/60),ss=sec%60;
     timerBox.style.display='block';
     timerText.textContent=String(mm).padStart(2,'0')+':'+String(ss).padStart(2,'0');
     timerBar.style.width=(100*Math.min(1,Math.max(0,(dur-rem)/dur))).toFixed(2)+'%';
   }else{
     timerBox.style.display='none';
   }
   $('footerRuntime').textContent=st.running?'работает':'остановлен';
   $('footerRuntime').style.color=st.running?'#15d876':'#8aa5b8';
   if(!st.running){
     if(!latest || latest.running!==false)updateHud({available:false,running:false,runtime_exit:st.runtime_exit||null});
   }else if(!latest || !latest.available){
     $('sceneXYZ').textContent='Ожидание WebSocket телеметрии… UDP rx='+String(st.udp_rx??0)+' bad='+String(st.udp_bad??0)+' · WS='+String(st.ws_clients??0);
   }
 }catch(e){}
}
setInterval(()=>{$('clock').textContent=new Date().toLocaleTimeString('ru-RU')},1000);
let voPreviewBusy=false;
async function refreshVoPreview(){
 if(voPreviewBusy)return;
 const img=$('voPreview'),st=$('voPreviewState');
 if(!img||!st)return;
 voPreviewBusy=true;
 try{
  const r=await fetch('/api/camera.jpg?t='+Date.now(),{cache:'no-store'});
  if(!r.ok){st.textContent='нет кадра';return}
  const blob=await r.blob();
  const url=URL.createObjectURL(blob),old=img.dataset.url;
  img.onload=()=>{if(old)URL.revokeObjectURL(old);st.textContent='LIVE'};
  img.src=url;img.dataset.url=url;
 }catch(e){st.textContent='нет кадра'}
 finally{voPreviewBusy=false}
}
loadConfig();initGL();connectTelemetryWs();refreshRuntimeStatus();refreshMessages();refreshFc();refreshJournal();refreshRunRecordStatus();refreshVoPreview();
setInterval(refreshRuntimeStatus,1000);setInterval(refreshMessages,1800);setInterval(refreshFc,1800);setInterval(refreshJournal,3000);setInterval(refreshRunRecordStatus,1500);setInterval(refreshVoPreview,200);
window.addEventListener('resize',()=>{let vm=window.visualizationMode||'simple';if(vm==='light'&&latest)drawLightScene(latest);else renderScene();if(latest)drawHistory(latest.history||[]);drawMotionCompare()});
</script>
</body>
</html>'''

class H(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass
    def send_json(self,obj,status=200):
        b=json.dumps(obj,ensure_ascii=False).encode()
        try:
            self.send_response(status);self.send_header("Content-Type","application/json; charset=utf-8")
            self.send_header("Content-Length",str(len(b)));self.end_headers();self.wfile.write(b)
        except (BrokenPipeError,ConnectionResetError):
            return
    def body_json(self):
        n=int(self.headers.get("Content-Length","0") or 0)
        return json.loads(self.rfile.read(n).decode("utf-8")) if n else {}
    def do_GET(self):
        p=urlparse(self.path).path
        try:
            if p=="/":
                b=HTML.encode();self.send_response(200);self.send_header("Content-Type","text/html; charset=utf-8");self.send_header("Content-Length",str(len(b)));self.end_headers();self.wfile.write(b)
            elif p=="/ws/telemetry":
                websocket_session(self)
                return
            elif p=="/charuco.pdf":
                fp=WEB_ASSETS/"charuco.pdf"
                data=fp.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type","application/pdf")
                if "download=1" in self.path:
                    self.send_header("Content-Disposition",'attachment; filename="charuco.pdf"')
                else:
                    self.send_header("Content-Disposition",'inline; filename="charuco.pdf"')
                self.send_header("Cache-Control","no-cache")
                self.send_header("Content-Length",str(len(data)))
                self.end_headers();self.wfile.write(data)
            elif p=="/api/camera.jpg":
                data=None
                # Prefer the atomic /dev/shm snapshot produced by the VO
                # process. Fall back to legacy UDP preview packets.
                try:
                    st=PREVIEW_PATH.stat()
                    if (time.time()-st.st_mtime) <= 2.0:
                        data=PREVIEW_PATH.read_bytes()
                except (FileNotFoundError,OSError):
                    pass
                if not data:
                    with _lock:
                        data=bytes(_camera_jpeg) if _camera_jpeg else None
                        age_ms=(time.time()-_camera_last_wall)*1000.0 if _camera_last_wall else None
                    if age_ms is None or age_ms>2000:
                        data=None
                if not data:
                    self.send_error(503,"camera preview unavailable")
                else:
                    self.send_response(200)
                    self.send_header("Content-Type","image/jpeg")
                    self.send_header("Cache-Control","no-store, no-cache, must-revalidate")
                    self.send_header("Content-Length",str(len(data)))
                    self.end_headers();self.wfile.write(data)
            elif p=="/api/config":
                self.send_json({"runtime":load_config(),"geometry":load_json(GEOMETRY,{}),"fc_profile":load_json(FC_PROFILE,{})})
            elif p=="/api/status":
                with _lock:
                    age_ms=(time.time()-_live_last_wall)*1000.0 if _live_last_wall else None
                with _lock:
                    test_duration_s=_runtime_test_duration_s
                    test_remaining_s=max(0.0,_runtime_test_deadline_mono-time.monotonic()) if running() and _runtime_test_deadline_mono>0.0 else None
                self.send_json({"running":running(),"pid":_proc.pid if running() else None,"transport":"websocket","live_age_ms":age_ms,"ws_clients":len(_ws_clients),"udp_rx":_live_udp_rx,"udp_bad":_live_udp_bad,"runtime_exit":runtime_exit_info(),"test_duration_s":test_duration_s or None,"test_remaining_s":test_remaining_s})
            elif p=="/api/telemetry":
                self.send_json(telemetry())
            elif p=="/api/log":
                self.send_json({"text":log_tail()})
            elif p=="/api/fc":
                self.send_json(fc_control("status"))
            elif p=="/api/fc/params":
                names=profile_param_names()
                self.send_json({"values":read_fc_params(names),"profile":load_json(FC_PROFILE,{})})
            elif p=="/api/geometry":
                self.send_json({"values":read_fc_params(GEOMETRY_PARAMS),"local":load_json(GEOMETRY,{})})
            elif p=="/api/messages":
                with _lock: self.send_json({"events":list(_messages)})
            elif p=="/api/journal":
                self.send_json({"events":journal_events()})
            elif p=="/api/run-record/status":
                self.send_json(run_record_status())
            elif p=="/api/run-logs":
                self.send_json({"logs":list_run_records()})
            elif p=="/api/run-logs/download":
                q=parse_qs(urlparse(self.path).query)
                names=(q.get("names",[""])[0]).split("\n")
                files=_selected_run_files(names)
                if len(files)==1:
                    fp=files[0];data=fp.read_bytes();filename=fp.name;ctype="text/csv; charset=utf-8"
                else:
                    bio=io.BytesIO()
                    with zipfile.ZipFile(bio,"w",zipfile.ZIP_DEFLATED) as z:
                        for fp in files:z.write(fp,arcname=fp.name)
                    data=bio.getvalue();filename="monkeysStab_logs.zip";ctype="application/zip"
                # BaseHTTPRequestHandler encodes header values as latin-1.
                # Keep the legacy filename ASCII-only and put the exact Unicode
                # run name (Cyrillic, ΔR, etc.) into RFC 5987 filename*.
                safe_ascii=re.sub(r'[^A-Za-z0-9._-]+','_',filename)
                if not safe_ascii or safe_ascii in ('.','..'):
                    safe_ascii='run_log.csv' if len(files)==1 else 'monkeysStab_logs.zip'
                disposition='attachment; filename="'+safe_ascii+'"; filename*=UTF-8\'\''+quote(filename,safe='')
                self.send_response(200);self.send_header("Content-Type",ctype)
                self.send_header("Content-Disposition",disposition)
                self.send_header("Content-Length",str(len(data)));self.end_headers();self.wfile.write(data)
            else:self.send_json({"error":"not found"},404)
        except Exception as e:self.send_json({"error":str(e)},500)
    def do_POST(self):
        p=urlparse(self.path).path
        try:
            if p=="/api/config":
                if running(): raise RuntimeError("Остановите flight runtime перед изменением стартовых параметров")
                self.send_json({"ok":True,"runtime":save_config(self.body_json())})
            elif p=="/api/start": forensic_action("WEB_RUNTIME_START");self.send_json(start_runtime())
            elif p=="/api/restart-fast": forensic_action("WEB_RUNTIME_RESTART");self.send_json(fast_restart_runtime())
            elif p=="/api/stop": forensic_action("WEB_RUNTIME_STOP");self.send_json(stop_runtime())
            elif p=="/api/zero": set_zero();self.send_json({"ok":True})
            elif p=="/api/run-record/start": self.send_json(start_run_record())
            elif p=="/api/run-record/stop": self.send_json(stop_run_record())
            elif p=="/api/run-record/finalize": self.send_json(finalize_run_record(self.body_json().get("name","")))
            elif p=="/api/fc/arm":
                forensic_action("WEB_ARM_PRESS");out=fc_control("arm");log_event("WARN","ARM подтверждён FC");forensic_action("WEB_ARM_OK");self.send_json(out)
            elif p=="/api/fc/disarm":
                forensic_action("WEB_DISARM_PRESS");out=fc_control("disarm");log_event("INFO","DISARM подтверждён FC");forensic_action("WEB_DISARM_OK");self.send_json(out)
            elif p=="/api/fc/mode":
                mode=str(self.body_json().get("mode","")).lower()
                if mode not in ("stabilize","poshold"):
                    raise ValueError("Разрешены только Stabilize и PosHold")
                forensic_action("WEB_MODE_PRESS",mode);out=fc_control("mode",mode);log_event("INFO","Режим FC -> "+mode);forensic_action("WEB_MODE_OK",mode);self.send_json(out)
            elif p=="/api/fc/takeoff":
                alt=float(self.body_json().get("alt_m",0.0))
                if not (0.10 <= alt <= 10.0):
                    raise ValueError("Высота TAKEOFF должна быть 0.10..10.0 м")
                forensic_action("WEB_TAKEOFF_PRESS",f"{alt:.3f}");out=fc_control("takeoff",f"{alt:.3f}")
                log_event("WARN",f"TAKEOFF принят FC: +{alt:.2f} м");forensic_action("WEB_TAKEOFF_OK",f"{alt:.3f}")
                self.send_json(out)
            elif p=="/api/fc/land":
                forensic_action("WEB_LAND_PRESS");out=fc_control("mode","land")
                log_event("WARN","LAND подтверждён FC");forensic_action("WEB_LAND_OK")
                self.send_json(out)
            elif p=="/api/fc/params":
                self.send_json({"ok":True,"values":set_profile_params(self.body_json().get("values",{}))})
            elif p=="/api/geometry":
                self.send_json({"ok":True,"values":set_geometry(self.body_json().get("values",{}))})
            elif p=="/api/system/visualization":
                mode=str(self.body_json().get("mode","simple"))
                if mode not in ("simple","light"): raise ValueError("Разрешены только simple и light")
                cfg=save_config({"visualization_mode":mode});log_event("INFO","Визуализация -> "+mode)
                self.send_json({"ok":True,"runtime":cfg})
            else:self.send_json({"error":"not found"},404)
        except Exception as e:self.send_json({"error":str(e)},400)

def ips():
    out=[]
    try:
        for info in socket.getaddrinfo(socket.gethostname(),None,socket.AF_INET):
            ip=info[4][0]
            if not ip.startswith("127.") and ip not in out: out.append(ip)
    except Exception: pass
    return out

def _web_shutdown_signal(signum,frame):
    # SIGTERM must run the normal finally cleanup instead of orphaning the
    # router/blackbox/runtime child sessions.
    raise KeyboardInterrupt

if __name__=="__main__":
    import argparse
    ap=argparse.ArgumentParser()
    ap.add_argument("--host",default="0.0.0.0")
    ap.add_argument("--port",type=int,default=8080)
    a=ap.parse_args()
    signal.signal(signal.SIGTERM,_web_shutdown_signal)
    RUN_ROOT.mkdir(parents=True,exist_ok=True)
    print("="*70)
    print("monkeysStab WEB")
    print(f"Локально: http://127.0.0.1:{a.port}")
    for ip in ips(): print(f"С компьютера в той же сети: http://{ip}:{a.port}")
    print("Web-интерфейс: runtime + FC ARM/DISARM + Stabilize/PosHold/Loiter.")
    print("="*70,flush=True)
    try:
        # Fail before touching router/blackbox/flight runtime if another Web UI
        # instance already owns the requested HTTP port.
        probe=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        try:
            probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            probe.bind((a.host,a.port))
        except OSError as e:
            raise RuntimeError(f"Web UI не может занять {a.host}:{a.port}: {e}") from e
        finally:
            probe.close()
        ensure_router()
        start_live_udp_listener()
        start_recovery_watchdog()
        start_statustext_monitor()
        try:
            start_fc_blackbox()
        except RuntimeError as e:
            # continuous_fc.csv is diagnostic only.  An orphan logger may
            # legitimately still own its flock after a previous Web process
            # was interrupted; do not make the flight runtime unavailable.
            if "code 3" in str(e):
                log_event("WARN","FC blackbox уже запущен; продолжаю без второго экземпляра")
                print("FC blackbox: УЖЕ ЗАПУЩЕН (использую существующий)",flush=True)
            else:
                raise
        log_event("INFO","Web UI запущен")
        print("MAVLink router: ГОТОВ, Mission Planner UDP 14550",flush=True)
        # Normal operating mode: starting the Web UI also starts the flight
        # runtime. start_runtime() already performs the runtime-side startup
        # checks, so do not add a second preflight/test cycle here.
        try:
            start_runtime()
            print("Flight runtime: АВТОЗАПУСК ГОТОВ",flush=True)
        except Exception as e:
            # Keep Web UI alive so the operator can inspect the exact startup
            # error and retry manually after fixing its cause.
            log_event("ERROR","Автозапуск flight runtime: "+str(e))
            print("Flight runtime: АВТОЗАПУСК ОШИБКА: "+str(e),flush=True)
        ThreadingHTTPServer((a.host,a.port),H).serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if running(): stop_runtime()
        stop_statustext_monitor()
        stop_live_udp_listener()
        stop_fc_blackbox()
        stop_router()
