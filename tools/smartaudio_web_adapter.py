#!/usr/bin/env python3
"""SmartAudio V2.1 adapter for future VTX web integration.

No frequency/power writes are implemented here.
The low-level driver owns the shared transaction lock.
"""

from smartaudio_driver import read

SOURCE = "SMARTAUDIO_V21_CONFIRMED"


def state():
    result = read(attempts=3)

    if result.get("ok") is not True:
        raise RuntimeError(
            "SmartAudio GET SETTINGS failed: "
            + repr(result.get("errors", []))
        )

    frequency = result.get("frequency_mhz")
    power_levels = result.get("power_levels_dbm")

    if type(frequency) is not int or not 5000 <= frequency <= 6000:
        raise RuntimeError("Invalid confirmed frequency")

    if (
        not isinstance(power_levels, list)
        or not 1 <= len(power_levels) <= 16
        or not all(type(v) is int for v in power_levels)
    ):
        raise RuntimeError("Invalid SmartAudio power table")

    return {
        "ok": True,
        "source": SOURCE,
        "protocol": "SMARTAUDIO_V21",
        "transport": "GPIO12_ONEWIRE",
        "frequency_mhz": frequency,
        "power_raw": result["power_raw"],
        "mode_raw": result["mode_raw"],
        "current_power_dbm": result["current_power_dbm"],
        "power_levels_dbm": power_levels,
        "power_level_count": len(power_levels),
        "power_control_available": False,
        "frequency_control_available": True,
        "crc_ok": result["crc_ok"],
        "confirmed_at": result["confirmed_at"],
    }


if __name__ == "__main__":
    import json
    import sys

    try:
        print(json.dumps(state(), ensure_ascii=False, indent=2))
    except Exception as exc:
        print(
            json.dumps({"ok": False, "error": str(exc)}),
            file=sys.stderr,
        )
        sys.exit(1)
