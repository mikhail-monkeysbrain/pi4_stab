#!/usr/bin/env python3
import argparse
import subprocess
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    args = parser.parse_args()

    device = Path(args.device)

    if not device.exists():
        raise SystemExit(f"Устройство не найдено: {device}")

    commands = [
        ["v4l2-ctl", "-d", str(device), "--all"],
        ["v4l2-ctl", "-d", str(device), "--list-formats-ext"],
        ["v4l2-ctl", "-d", str(device), "--list-ctrls"],
    ]

    for command in commands:
        print("\n===== " + " ".join(command) + " =====", flush=True)
        result = subprocess.run(command, check=False)
        if result.returncode:
            print(f"Команда завершилась с кодом {result.returncode}")

if __name__ == "__main__":
    main()
