#!/usr/bin/env python3
"""Read-only live barometric altitude from ArduPilot via MAVLink."""
import argparse
import math
import sys
import time
from pymavlink import mavutil

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", default="tcp:127.0.0.1:5760")
    args = parser.parse_args()
    print("Подключение к FC:", args.connect, flush=True)
    link = mavutil.mavlink_connection(args.connect, autoreconnect=True)
    link.wait_heartbeat(timeout=15)
    # Request pressure and EKF altitude telemetry without changing FC parameters.
    for message_id in (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
                       mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, 100000, 0, 0, 0, 0, 0)
    print("Heartbeat получен. Запрошены SCALED_PRESSURE и GLOBAL_POSITION_INT (10 Гц). Ctrl+C — выход.", flush=True)
    baseline = None
    last_pressure = None
    last_ekf = None
    last_print = 0.0
    last_baro_at = 0.0
    while True:
        msg = link.recv_match(type=["SCALED_PRESSURE", "SCALED_PRESSURE2", "SCALED_PRESSURE3", "GLOBAL_POSITION_INT"], blocking=True, timeout=0.2)
        now = time.monotonic()
        if msg is not None:
            kind = msg.get_type()
            if kind.startswith("SCALED_PRESSURE"):
                p = float(msg.press_abs)
                if math.isfinite(p) and p > 0:
                    if baseline is None:
                        baseline = p
                    last_pressure = p
                    last_baro_at = now
            elif kind == "GLOBAL_POSITION_INT":
                last_ekf = float(msg.relative_alt) / 1000.0
        if now - last_print >= 0.2:
            last_print = now
            if last_pressure is None or now - last_baro_at > 3:
                line = "Нет свежих данных SCALED_PRESSURE (проверьте MAVLink stream)"
            else:
                delta = 44330.0 * (1.0 - (last_pressure / baseline) ** 0.190294957)
                ekf = f"{last_ekf:+.3f} м" if last_ekf is not None else "нет данных"
                line = f"ΔBARO {delta:+.3f} м | P {last_pressure:.2f} гПа | EKF ALT {ekf}"
            sys.stdout.write("\r" + line.ljust(110))
            sys.stdout.flush()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nОстановлено")
    except Exception as exc:
        print(f"\nОшибка: {exc}", file=sys.stderr)
        sys.exit(1)
