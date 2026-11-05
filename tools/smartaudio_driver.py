#!/usr/bin/env python3
"""Read-only SmartAudio V2.1 driver for Raspberry Pi 5 GPIO12."""

import argparse
import fcntl
import json
import re
import subprocess
import sys
import time
from pathlib import Path

from smartaudio_protocol import decode_settings

BINARY = Path("/usr/local/sbin/monkeysstab-smartaudio-gpiomem")
LOCK = "/tmp/monkeysstab_tramp_transaction.lock"


def read(attempts=3):
    errors = []

    with open(LOCK, "a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)

        for attempt in range(1, attempts + 1):
            try:
                proc = subprocess.run(
                    ["sudo", "-n", str(BINARY)],
                    capture_output=True,
                    text=True,
                    timeout=3,
                    check=False,
                )

                if proc.returncode != 0:
                    raise RuntimeError(
                        f"exit={proc.returncode}: "
                        f"{proc.stdout[-500:]} {proc.stderr[-500:]}"
                    )

                match = re.search(
                    r"^RX:\s*([0-9A-Fa-f ]+)$",
                    proc.stdout,
                    re.MULTILINE,
                )

                if not match:
                    raise ValueError("RX line missing")

                raw = bytes.fromhex(match.group(1))

                # Find a complete, CRC-validated SmartAudio frame.
                result = None
                for i in range(len(raw) - 4):
                    if raw[i:i + 2] != b"\xAA\x55":
                        continue

                    try:
                        result = decode_settings(raw[i:])
                        break
                    except ValueError:
                        continue

                if result is None:
                    raise ValueError(
                        "No valid SmartAudio V2.1 frame"
                    )

                result["attempt"] = attempt
                result["confirmed_at"] = time.time()

                return result

            except (
                OSError,
                ValueError,
                RuntimeError,
                subprocess.TimeoutExpired,
            ) as exc:
                errors.append(str(exc))

            if attempt < attempts:
                time.sleep(0.2)

    return {
        "ok": False,
        "protocol": "SMARTAUDIO_V21",
        "errors": errors,
        "attempts": attempts,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--attempts", type=int, default=3
    )
    args = parser.parse_args()

    if not 1 <= args.attempts <= 10:
        parser.error("attempts must be 1..10")

    if not BINARY.is_file():
        print(json.dumps({
            "ok": False,
            "error": f"Binary missing: {BINARY}"
        }))
        return 2

    result = read(args.attempts)
    print(json.dumps(
        result, ensure_ascii=False, indent=2
    ))

    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
