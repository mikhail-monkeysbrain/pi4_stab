#!/usr/bin/env python3
"""
Foxeer IRC Tramp one-wire read driver.

GPIO12 -> UART5 TX / GPIO input RX
Uses the existing tramp_gpiomem binary.
Does not modify the optical-flow pipeline.
"""

import argparse
import fcntl
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BINARY = Path("/usr/local/sbin/monkeysstab-tramp-gpiomem")
LOCKFILE = "/tmp/monkeysstab_tramp_uart5.lock"

COMMANDS = {"r", "v", "s"}


def parse_frame(output, command):
    match = re.search(r"^RX:\s*(.*)$", output, re.MULTILINE)

    if not match:
        raise ValueError("RX line missing")

    try:
        frame = bytes.fromhex(match.group(1))
    except ValueError:
        raise ValueError("Invalid RX hex")

    if len(frame) != 16:
        raise ValueError(f"Invalid frame length: {len(frame)}")

    if frame[0] != 0x0F:
        raise ValueError("Invalid start byte")

    if frame[1] != ord(command):
        raise ValueError("Unexpected response command")

    if frame[15] != 0:
        raise ValueError("Invalid end byte")

    checksum = sum(frame[1:14]) & 0xFF

    if checksum != frame[14]:
        raise ValueError(
            f"Checksum mismatch: expected {checksum:02X}, "
            f"received {frame[14]:02X}"
        )

    return frame


def decode(frame, command):
    def u16(offset):
        return int.from_bytes(
            frame[offset:offset + 2], "little"
        )

    if command == "r":
        return {
            "frequency_min_mhz": u16(2),
            "frequency_max_mhz": u16(4),
            "power_limit_raw": u16(6),
        }

    if command == "v":
        return {
            "frequency_mhz": u16(2),
            "power_setting_raw": u16(4),
            "field_8_raw": u16(8),
        }

    if command == "s":
        return {
            "status_raw": list(frame[2:14]),
            "field_6_raw": u16(6),
        }

    return {}


def read(command, attempts=3):
    errors = []

    with open(LOCKFILE, "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)

        for attempt in range(1, attempts + 1):
            try:
                result = subprocess.run(
                    ["sudo", "-n", str(BINARY), command],
                    capture_output=True,
                    text=True,
                    timeout=3,
                    check=False,
                )

                if result.returncode != 0:
                    raise RuntimeError(
                        f"Receiver exit={result.returncode}: "
                        f"{result.stdout.strip()} "
                        f"{result.stderr.strip()}"
                    )

                frame = parse_frame(result.stdout, command)

                return {
                    "ok": True,
                    "protocol": "IRC_TRAMP",
                    "transport": "GPIO12_ONEWIRE",
                    "command": command,
                    "attempt": attempt,
                    "raw_hex": frame.hex(" ").upper(),
                    "data": decode(frame, command),
                }

            except (
                ValueError,
                RuntimeError,
                subprocess.TimeoutExpired,
                OSError,
            ) as exc:
                errors.append(str(exc))

            if attempt < attempts:
                time.sleep(0.2)

    return {
        "ok": False,
        "protocol": "IRC_TRAMP",
        "command": command,
        "attempts": attempts,
        "errors": errors,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "command", choices=sorted(COMMANDS)
    )
    parser.add_argument(
        "--attempts", type=int, default=3
    )
    args = parser.parse_args()

    if not 1 <= args.attempts <= 10:
        parser.error("attempts must be 1..10")

    if not BINARY.is_file():
        print(json.dumps({
            "ok": False,
            "error": f"Binary not found: {BINARY}",
        }))
        return 2

    result = read(args.command, args.attempts)

    print(json.dumps(
        result, ensure_ascii=False, indent=2
    ))

    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
