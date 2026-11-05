#!/usr/bin/env python3
"""VTX bridge: serialize all web requests with existing transaction lock."""
import fcntl
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOCK = '/tmp/monkeysstab_tramp_transaction.lock'

def _call(script, *args):
    p = subprocess.run([sys.executable, str(HERE / script), *map(str, args)],
                       capture_output=True, text=True, timeout=18, check=False)
    try:
        result = json.loads(p.stdout)
    except (ValueError, TypeError):
        raise RuntimeError('VTX helper failed: ' + (p.stderr or p.stdout)[-600:])
    if p.returncode or not result.get('ok'):
        raise RuntimeError('VTX: ' + json.dumps(result, ensure_ascii=False)[-900:])
    return result


STATE_FILE = Path('/home/vio/jtzero_vtx_state.json')

def _save_state(frequency, power):
    import os
    import time
    if not isinstance(frequency, int) or not 5100 <= frequency <= 6000:
        return
    if not isinstance(power, int) or not 0 <= power <= 65535:
        return
    data = {
        'ok': True,
        'source': 'IRC_TRAMP_CONFIRMED',
        'frequency_mhz': frequency,
        'power_setting_raw': power,
        'confirmed_at': time.time()
    }
    temp = STATE_FILE.with_suffix('.tmp')
    temp.write_text(json.dumps(data))
    os.replace(temp, STATE_FILE)

def state():
    # Snapshot uses read()'s UART lock; transaction lock keeps web writes exclusive.
    with open(LOCK, 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        result = _call('tramp_snapshot.py')
        _save_state(result['frequency_mhz'], result['power_setting_raw'])
        return result

def set_frequency(freq):
    if type(freq) is not int or not 5100 <= freq <= 6000:
        raise ValueError('Frequency must be integer 5100..6000 MHz')
    # Existing setter holds the transaction lock itself; do not acquire it here.
    result = _call('tramp_set_confirmed.py', '--freq', freq)
    # The setter confirms frequency only. Read a fresh full snapshot after it
    # releases its transaction lock; never reuse cached power.
    STATE_FILE.unlink(missing_ok=True)
    state()
    return result

def set_power(power):
    if type(power) is not int or power not in (25, 100, 200, 400, 600):
        raise ValueError('Allowed VTX raw power settings: 25, 100, 200, 400, 600')
    result = _call('tramp_power_confirmed.py', '--power', power)
    after = result.get('after')
    if isinstance(after, dict):
        _save_state(after.get('frequency_mhz'), after.get('power_raw'))
    return result

# ===== SMARTAUDIO V2.1 BACKEND =====
# Explicit selection; legacy Tramp functions above remain untouched.
import os as _vtx_os
_tramp_state = state
_tramp_set_frequency = set_frequency
_tramp_set_power = set_power

def _protocol():
    value = _vtx_os.environ.get("MONKEYS_VTX_PROTOCOL", "SMARTAUDIO").upper()
    if value not in ("SMARTAUDIO", "TRAMP"):
        raise ValueError("MONKEYS_VTX_PROTOCOL must be SMARTAUDIO or TRAMP")
    return value

def _smart_state():
    from smartaudio_web_adapter import state as read_state
    result = read_state()
    data = {
        "ok": True, "source": result["source"],
        "frequency_mhz": result["frequency_mhz"],
        "power_raw": result["power_raw"],
        "current_power_dbm": result["current_power_dbm"],
        "power_levels_dbm": result["power_levels_dbm"],
        "power_level_count": result["power_level_count"],
        "protocol": "SMARTAUDIO_V21",
        "confirmed_at": result["confirmed_at"],
    }
    temp = STATE_FILE.with_suffix(".tmp")
    temp.write_text(json.dumps(data))
    _vtx_os.replace(temp, STATE_FILE)
    return result

def state():
    if _protocol() == "TRAMP":
        return _tramp_state()
    return _smart_state()

def set_frequency(freq):
    if _protocol() == "TRAMP":
        from vtx_fc_safety import require_disarmed
        require_disarmed()
        return _tramp_set_frequency(freq)
    from smartaudio_set_confirmed import set_frequency as smart_set
    result = smart_set(freq)
    _smart_state()
    return result

def set_power(power):
    if _protocol() == "TRAMP":
        from vtx_fc_safety import require_disarmed
        require_disarmed()
        return _tramp_set_power(power)
    from smartaudio_set_confirmed import set_power as smart_set_power
    result = smart_set_power(power)
    _smart_state()
    return result
