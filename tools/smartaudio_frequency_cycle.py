#!/usr/bin/env python3
"""Controlled SmartAudio A1 -> A2 -> A1 hardware test."""

import argparse
import fcntl
import re
import subprocess
import sys
import time
from pathlib import Path

from smartaudio_protocol import decode_settings

BINARY = "/tmp/monkeysstab-smartaudio-setfreq-v2"
LOCK = "/tmp/monkeysstab_tramp_transaction.lock"
ORIGINAL = 5865
TARGET = 5845


def run_binary(*args):
    p = subprocess.run(
        [BINARY, *map(str, args)],
        capture_output=True,
        text=True,
        timeout=4,
    )
    return p


def read_frequency():
    errors = []

    for attempt in range(3):
        p = run_binary()

        if p.returncode != 0:
            errors.append(f"exit={p.returncode}")
            continue

        m = re.search(
            r"^RX:\s*([0-9A-Fa-f ]+)$",
            p.stdout,
            re.MULTILINE,
        )

        if not m:
            errors.append("RX missing")
            continue

        raw = bytes.fromhex(m.group(1))

        for i in range(len(raw) - 4):
            if raw[i:i+2] != b"\xAA\x55":
                continue

            try:
                state = decode_settings(raw[i:])
                frequency = state["frequency_mhz"]
                print(
                    f"READ: {frequency} MHz "
                    f"CRC OK (attempt {attempt+1})",
                    flush=True,
                )
                return frequency
            except ValueError:
                pass

        errors.append("No valid settings frame")
        time.sleep(0.2)

    raise RuntimeError("GET SETTINGS failed: " + "; ".join(errors))


def set_frequency(frequency):
    print(f"SET FREQUENCY: {frequency} MHz", flush=True)

    p = run_binary("--set-frequency", frequency)

    for line in p.stdout.splitlines():
        if (
            line.startswith("TX:")
            or line.startswith("FRAME ")
            or line.startswith("CRC ")
        ):
            print(line, flush=True)

    if p.returncode != 0:
        print(
            f"SET returned exit={p.returncode}; "
            "the VTX may still have changed frequency",
            flush=True,
        )

    # Never interpret the SET response as proof of success.
    return p.returncode


def confirm(expected):
    for attempt in range(3):
        try:
            actual = read_frequency()
            if actual == expected:
                return True

            print(
                f"EXPECTED {expected}, ACTUAL {actual}",
                flush=True,
            )
        except Exception as exc:
            print(f"READ ERROR: {exc}", flush=True)

        time.sleep(0.3)

    return False


def gpio_check():
    p = subprocess.run(
        ["pinctrl", "get", "12"],
        capture_output=True,
        text=True,
    )
    print("GPIO12:", p.stdout.strip(), flush=True)

    if p.returncode != 0 or "TXD4" not in p.stdout:
        raise RuntimeError("GPIO12 not confirmed as TXD4")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually change VTX frequency",
    )
    args = parser.parse_args()

    if not Path(BINARY).is_file():
        raise RuntimeError("SmartAudio binary missing")

    if not args.execute:
        print("DRY RUN: no GPIO access, no frequency changes")
        print(f"Binary: {BINARY}")
        print(f"Planned: {ORIGINAL} -> {TARGET} -> {ORIGINAL}")
        return 0

    if __import__("os").geteuid() != 0:
        raise RuntimeError("Run hardware cycle as root")

    import os

    fd = os.open(LOCK, os.O_RDONLY)
    with os.fdopen(fd, "r") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

        gpio_check()

        initial = read_frequency()
        if initial != ORIGINAL:
            raise RuntimeError(
                f"ABORT: initial frequency {initial}, "
                f"expected {ORIGINAL}"
            )

        changed = False
        target_confirmed = False
        restored = False

        try:
            # From this point a command may change the VTX,
            # even if the command reports an error.
            changed = True
            set_frequency(TARGET)
            time.sleep(0.4)

            target_confirmed = confirm(TARGET)

            if not target_confirmed:
                raise RuntimeError(
                    "5845 MHz was not confirmed"
                )

            print("A2 CONFIRMED", flush=True)

        finally:
            if changed:
                print("RESTORING 5865 MHz", flush=True)

                for recovery_attempt in range(3):
                    try:
                        set_frequency(ORIGINAL)
                        time.sleep(0.4)

                        if confirm(ORIGINAL):
                            restored = True
                            break
                    except Exception as exc:
                        print(
                            f"RECOVERY ERROR: {exc}",
                            flush=True,
                        )

                try:
                    gpio_check()
                except Exception as exc:
                    print(f"GPIO WARNING: {exc}", flush=True)

                if not restored:
                    print(
                        "CRITICAL: ORIGINAL FREQUENCY "
                        "NOT CONFIRMED",
                        flush=True,
                    )

        if not restored:
            return 3

        if not target_confirmed:
            return 2

        print(
            "PASS: 5865 -> 5845 -> 5865 CONFIRMED",
            flush=True,
        )
        return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr, flush=True)
        sys.exit(1)
