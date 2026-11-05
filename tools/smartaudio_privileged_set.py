#!/usr/bin/env python3
"""Privileged SmartAudio frequency writer.

Standalone: no imports from user-writable project modules.
"""

import json
import os
import subprocess
import sys

BINARY = "/usr/local/sbin/monkeysstab-smartaudio-gpiomem"
FC_BINARY = "/home/vio/Desktop/monkeysStab_pi4/build/fc_control"
FC_ENDPOINT = "tcp://127.0.0.1:5760"

BANDS = (
    (5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725),
    (5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866),
    (5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945),
    (5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880),
    (5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917),
)

ALLOWED = frozenset(f for band in BANDS for f in band)


def main():
    if len(sys.argv) != 2:
        raise RuntimeError("Expected exactly one frequency")

    value = sys.argv[1]

    if not value.isascii() or not value.isdecimal():
        raise RuntimeError("Invalid frequency format")

    frequency = int(value)

    if frequency not in ALLOWED:
        raise RuntimeError("Frequency not in channel table")

    fc = subprocess.run(
        [FC_BINARY, FC_ENDPOINT, "status"],
        capture_output=True,
        text=True,
        timeout=6,
        check=True,
        env={"PATH": "/usr/bin:/bin"},
    )

    status = json.loads(fc.stdout.strip().splitlines()[-1])

    if (
        status.get("ok") is not True
        or status.get("armed") is not False
        or status.get("sysid") != 1
        or status.get("compid") != 1
    ):
        raise RuntimeError("FC DISARMED not confirmed")

    result = subprocess.run(
        [BINARY, "--set-frequency", str(frequency)],
        capture_output=True,
        text=True,
        timeout=4,
        check=False,
        env={"PATH": "/usr/bin:/bin"},
    )

    if result.returncode != 0:
        raise RuntimeError(
            "SmartAudio SET failed: " + result.stderr[-300:]
        )

    print(json.dumps({
        "ok": True,
        "frequency_mhz": frequency,
        "status": "set_sent_readback_required",
    }))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR:", exc, file=sys.stderr)
        sys.exit(1)
