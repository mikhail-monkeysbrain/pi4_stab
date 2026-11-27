#!/usr/bin/env python3

import argparse
import json
import struct
import time
from pathlib import Path
from pymavlink import mavutil

BANDS = {
    1: [5865,5845,5825,5805,5785,5765,5745,5725],
    2: [5733,5752,5771,5790,5800+11,5830,5847,5866],
    3: [5705,5685,5665,5645,5885,5905,5925,5945],
    4: [5740,5760,5780,5800,5820,5840,5860,5880],
    5: [5658,5695,5732,5769,5806,5843,5880,5917],
}

BANDS[2] = [5733,5752,5771,5790,5809,5828,5847,5866]

POWER_MAP = {
    25: 25,
    100: 200,
    200: 500,
    400: 1500,
    600: 2500,
}

def read_vtx(path):
    data = json.loads(Path(path).read_text())

    if data.get("ok") is not True:
        raise ValueError("VTX state not confirmed")

    if data.get("source") != "IRC_TRAMP_CONFIRMED":
        raise ValueError("Unexpected VTX state source")

    freq = int(data["frequency_mhz"])
    power = POWER_MAP[int(data["power_setting_raw"])]

    for band, frequencies in BANDS.items():
        if freq in frequencies:
            return band, frequencies.index(freq) + 1, power

    raise ValueError(f"Unknown frequency: {freq}")

def send_vtx(link, band, channel, power):
    payload = (
        b"JV"
        + bytes((1, band, channel))
        + struct.pack("<H", power)
    )

    link.mav.data96_send(
        78,
        len(payload),
        list(payload.ljust(96, b"\x00"))
    )

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--endpoint",
        default="tcp:127.0.0.1:5760"
    )
    parser.add_argument("--comp-ready", type=int, default=0)
    parser.add_argument("--flow", type=int, default=0)
    parser.add_argument(
        "--vtx-state-file",
        default="/home/vio/jtzero_vtx_state.json"
    )
    parser.add_argument("--repeat", action="store_true")

    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.endpoint,
        source_system=42,
        source_component=191
    )

    print("VTX OSD: connected", flush=True)

    previous = None

    while True:
        try:
            state = read_vtx(args.vtx_state_file)
            send_vtx(link, *state)

            if state != previous:
                print(f"VTX={state}", flush=True)
                previous = state

        except (ValueError, KeyError, OSError) as exc:
            print(f"VTX OSD ERROR: {exc}", flush=True)
            if not args.repeat:
                raise

        if not args.repeat:
            break

        time.sleep(1.0)

if __name__ == "__main__":
    main()
