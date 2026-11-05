#!/usr/bin/env python3

import argparse
import fcntl
import json
import subprocess
import sys
import time
from pathlib import Path

from tramp_driver import read

ROOT = Path(__file__).resolve().parent
TX = ROOT / "tramp_vtx.py"
LOCK = "/tmp/monkeysstab_tramp_transaction.lock"

ALLOWED_POWER = (25, 100, 200, 400, 600)


def get_state():
    result = read("v", attempts=3)

    if not result.get("ok"):
        raise RuntimeError(
            "Cannot read VTX: " + json.dumps(result)
        )

    data = result["data"]

    return {
        "frequency_mhz": data["frequency_mhz"],
        "power_raw": data["power_setting_raw"],
        "field_8_raw": data["field_8_raw"]
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--power",
        type=int,
        choices=ALLOWED_POWER,
        required=True
    )
    args = parser.parse_args()

    with open(LOCK, "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)

        before = get_state()

        if before["power_raw"] == args.power:
            print(json.dumps({
                "ok": True,
                "status": "ALREADY_SET",
                "power_requested": args.power,
                "before": before,
                "after": before,
                "command_sent": False
            }, indent=2))
            return 0

        tx = subprocess.run(
            [
                sys.executable,
                str(TX),
                "--power",
                str(args.power)
            ],
            capture_output=True,
            text=True,
            timeout=5,
            check=False
        )

        if tx.returncode != 0:
            raise RuntimeError(
                "Power TX failed: " +
                tx.stdout + tx.stderr
            )

        time.sleep(0.4)

        try:
            after = get_state()
        except Exception as exc:
            print(json.dumps({
                "ok": False,
                "status": "STATE_UNKNOWN",
                "power_requested": args.power,
                "before": before,
                "command_sent": True,
                "error": str(exc)
            }, indent=2))
            return 1

        confirmed = (
            after["power_raw"] == args.power
        )

        frequency_unchanged = (
            after["frequency_mhz"] ==
            before["frequency_mhz"]
        )

        ok = confirmed and frequency_unchanged

        print(json.dumps({
            "ok": ok,
            "status": (
                "CONFIRMED" if ok
                else "NOT_CONFIRMED"
            ),
            "power_requested": args.power,
            "power_before_raw": before["power_raw"],
            "power_after_raw": after["power_raw"],
            "frequency_unchanged": frequency_unchanged,
            "before": before,
            "after": after,
            "command_sent": True
        }, indent=2))

        return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(json.dumps({
            "ok": False,
            "status": "ERROR",
            "error": str(exc)
        }, indent=2))
        sys.exit(1)
