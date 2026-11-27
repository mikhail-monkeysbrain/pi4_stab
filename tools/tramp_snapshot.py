#!/usr/bin/env python3
import json
import sys
import time

from tramp_driver import read


def main():
    results = {}

    for command in ("r", "v", "s"):
        result = read(command, attempts=3)
        results[command] = result

        if command != "s":
            time.sleep(0.2)

    valid = {
        command: result["data"]
        for command, result in results.items()
        if result.get("ok")
    }

    output = {
        "ok": len(valid) == 3,
        "protocol": "IRC_TRAMP",
        "transport": "PI5_UART2_ONEWIRE",
        "frequency_mhz": valid.get("v", {}).get("frequency_mhz"),
        "power_setting_raw": valid.get("v", {}).get("power_setting_raw"),
        "power_field_8_raw": valid.get("v", {}).get("field_8_raw"),
        "status_field_6_raw": valid.get("s", {}).get("field_6_raw"),
        "frequency_min_mhz": valid.get("r", {}).get("frequency_min_mhz"),
        "frequency_max_mhz": valid.get("r", {}).get("frequency_max_mhz"),
        "power_limit_raw": valid.get("r", {}).get("power_limit_raw"),
        "responses": results,
    }

    print(json.dumps(output, indent=2, ensure_ascii=False))
    return 0 if output["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
