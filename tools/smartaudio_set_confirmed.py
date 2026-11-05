#!/usr/bin/env python3
"""Confirmed SmartAudio frequency setter.

Not connected to web/RC until ARM safety checks are integrated.
"""

import fcntl
import os
import subprocess
import time

from vtx_fc_safety import require_disarmed

BINARY = "/usr/local/sbin/monkeysstab-smartaudio-gpiomem"
LOCK = "/tmp/monkeysstab_tramp_transaction.lock"

BANDS = {
    "A": (5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725),
    "B": (5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866),
    "E": (5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945),
    "F": (5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880),
    "R": (5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917),
}

ALLOWED_FREQUENCIES = frozenset(
    frequency
    for frequencies in BANDS.values()
    for frequency in frequencies
)


def _read_unlocked():
    """Read while caller already owns the transaction lock.

    The existing smartaudio_driver.read() acquires its own lock,
    so it must not be called from here.
    """
    from smartaudio_protocol import decode_settings
    import re

    p = subprocess.run(
        ["sudo", "-n", BINARY],
        capture_output=True,
        text=True,
        timeout=4,
        check=False,
    )

    if p.returncode:
        raise RuntimeError(
            "GET SETTINGS failed: " + p.stderr[-400:]
        )

    match = re.search(
        r"^RX:\s*([0-9A-Fa-f ]+)$",
        p.stdout,
        re.MULTILINE,
    )

    if not match:
        raise RuntimeError("No RX frame")

    raw = bytes.fromhex(match.group(1))

    for i in range(len(raw) - 4):
        if raw[i:i + 2] != b"\xAA\x55":
            continue
        try:
            return decode_settings(raw[i:])
        except ValueError:
            continue

    raise RuntimeError("No valid SmartAudio settings frame")


def set_frequency(frequency):
    if type(frequency) is not int:
        raise ValueError("Frequency must be integer")

    if frequency not in ALLOWED_FREQUENCIES:
        raise ValueError("Frequency is not in the 40-channel table")

    # This module must only be invoked after an independent,
    # fresh FC DISARMED verification at the API/RC boundary.
    # Do not expose it directly as an HTTP or CLI write endpoint
    # before that safety gate is implemented.

    with open(LOCK, "a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)

        before = _read_unlocked()

        if before["frequency_mhz"] == frequency:
            return {
                "ok": True,
                "status": "already_confirmed",
                "frequency_mhz": frequency,
                "before": before,
                "after": before,
            }

        # Fresh FC HEARTBEAT immediately before the VTX write.
        # This is a point-in-time check, not an ARM interlock.
        require_disarmed()

        command = subprocess.run(
            [
                "sudo", "-n",
                "/usr/local/libexec/monkeysstab/smartaudio_privileged_set.py",
                str(frequency),
            ],
            capture_output=True,
            text=True,
            timeout=4,
            check=False,
        )

        if command.returncode != 0:
            raise RuntimeError("SmartAudio SET rejected: " + command.stderr[-400:])

        # The SET response is not proof of a successful change.
        # Confirm using independent GET SETTINGS.
        last_error = None

        for attempt in range(3):
            time.sleep(0.25)

            try:
                after = _read_unlocked()
                if after["frequency_mhz"] == frequency:
                    return {
                        "ok": True,
                        "status": "confirmed",
                        "frequency_mhz": frequency,
                        "before": before,
                        "after": after,
                        "set_exit_code": command.returncode,
                    }
                last_error = (
                    "Readback frequency: "
                    + str(after["frequency_mhz"])
                )
            except Exception as exc:
                last_error = str(exc)

        raise RuntimeError(
            "SmartAudio frequency not confirmed; "
            + str(last_error)
        )


# ===== JTZERO SMARTAUDIO CONFIRMED POWER =====

POWER_LEVELS = (14, 27, 30, 34)

POWER_HELPER = (
    "/usr/local/libexec/monkeysstab/"
    "smartaudio_privileged_power.py"
)


def set_power(power):
    """Set SmartAudio power with independent hardware readback."""

    if type(power) is not int or power not in POWER_LEVELS:
        raise ValueError("Invalid SmartAudio power level")

    # The privileged helper must be installed separately.
    if not os.path.isfile(POWER_HELPER):
        raise RuntimeError(
            "SmartAudio privileged power helper not installed"
        )

    with open(LOCK, "a+") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)

        before = _read_unlocked()

        if before["current_power_dbm"] == power:
            return {
                "ok": True,
                "status": "already_confirmed",
                "before": before,
                "after": before,
            }

        require_disarmed()

        index = POWER_LEVELS.index(power)

        command = subprocess.run(
            ["sudo", "-n", POWER_HELPER, str(index)],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )

        last_error = (
            command.stderr[-400:]
            if command.returncode
            else "readback mismatch"
        )

        for attempt in range(3):
            time.sleep(0.3)

            try:
                after = _read_unlocked()

                if (
                    after["current_power_dbm"] == power
                    and after["frequency_mhz"]
                    == before["frequency_mhz"]
                ):
                    return {
                        "ok": True,
                        "status": "confirmed",
                        "before": before,
                        "after": after,
                        "set_exit_code": command.returncode,
                    }

                last_error = (
                    "Unexpected VTX state: " + str(after)
                )

            except Exception as exc:
                last_error = str(exc)

        raise RuntimeError(
            "SmartAudio power not confirmed: " + last_error
        )

# ===== END JTZERO SMARTAUDIO CONFIRMED POWER =====
