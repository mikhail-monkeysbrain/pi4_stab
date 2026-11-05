#!/usr/bin/env python3
"""Fail-closed FC DISARMED verification for VTX operations."""

import json
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FC_SCRIPT = ROOT / "scripts" / "fc_control.sh"


class VtxSafetyError(RuntimeError):
    pass


def require_disarmed():
    """Obtain a new FC HEARTBEAT via the existing MAVLink router.

    Raises on ARM, unknown status, timeout or invalid response.
    This is a point-in-time check, not an ARM interlock.
    """
    if not FC_SCRIPT.is_file():
        raise VtxSafetyError("FC control script missing")

    try:
        p = subprocess.run(
            ["bash", str(FC_SCRIPT), "status"],
            cwd=str(ROOT),
            env=os.environ.copy(),
            capture_output=True,
            text=True,
            timeout=8,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise VtxSafetyError("FC status timeout") from exc

    if p.returncode != 0:
        raise VtxSafetyError(
            "FC status unavailable: " +
            (p.stderr or p.stdout)[-400:]
        )

    lines = p.stdout.strip().splitlines()

    if not lines:
        raise VtxSafetyError("FC status returned no data")

    try:
        status = json.loads(lines[-1])
    except (ValueError, TypeError) as exc:
        raise VtxSafetyError("Invalid FC status JSON") from exc

    if not isinstance(status, dict):
        raise VtxSafetyError("Invalid FC status structure")

    if status.get("ok") is not True:
        raise VtxSafetyError("FC status not confirmed")

    # Do not use bool(status.get("armed", False)).
    # Only literal JSON false is accepted.
    if status.get("armed") is not False:
        raise VtxSafetyError("FC is ARMED or ARM state unknown")

    sysid = status.get("sysid")
    compid = status.get("compid")

    if type(sysid) is not int or not 1 <= sysid <= 255:
        raise VtxSafetyError("Invalid FC system ID")

    if compid != 1:
        raise VtxSafetyError(
            f"Unexpected FC component ID: {compid}"
        )

    return {
        "ok": True,
        "armed": False,
        "sysid": sysid,
        "compid": compid,
        "mode": status.get("mode"),
    }
