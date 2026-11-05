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
TX_SCRIPT = ROOT / "tramp_vtx.py"
LOCKFILE = "/tmp/monkeysstab_tramp_transaction.lock"


def read_frequency():
    result = read("v", attempts=3)

    if not result.get("ok"):
        raise RuntimeError(
            "Cannot read VTX frequency: "
            + json.dumps(result)
        )

    frequency = result["data"]["frequency_mhz"]

    if not 5000 <= frequency <= 6000:
        raise RuntimeError(
            f"Unexpected frequency: {frequency}"
        )

    return frequency


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--freq", type=int, required=True)
    args = parser.parse_args()

    if not 5100 <= args.freq <= 6000:
        parser.error("Frequency must be 5100..6000 MHz")

    with open(LOCKFILE, "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)

        before = read_frequency()

        if before == args.freq:
            print(json.dumps({
                "ok": True,
                "status": "ALREADY_SET",
                "frequency_mhz": before,
                "changed": False
            }, indent=2))
            return 0

        result = subprocess.run(
            [
                sys.executable,
                str(TX_SCRIPT),
                "--freq",
                str(args.freq)
            ],
            capture_output=True,
            text=True,
            timeout=5,
            check=False
        )

        if result.returncode != 0:
            raise RuntimeError(
                "VTX write failed: "
                + result.stderr.strip()
            )

        time.sleep(0.3)

        after = read_frequency()

        confirmed = after == args.freq

        print(json.dumps({
            "ok": confirmed,
            "status": (
                "CONFIRMED" if confirmed
                else "NOT_CONFIRMED"
            ),
            "frequency_before_mhz": before,
            "frequency_requested_mhz": args.freq,
            "frequency_after_mhz": after,
            "changed": before != after
        }, indent=2))

        return 0 if confirmed else 1


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
