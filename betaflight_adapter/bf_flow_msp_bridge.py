#!/usr/bin/env python3
"""Convert BF Core samples to MSPv2 packets offline.

Reads BF_CORE_SAMPLE lines from stdin.
Prints encoded packets as hexadecimal text.
Never opens a UART or sends anything to a flight controller.
"""

import argparse
import math
import sys

from msp_encoder import AngularMotionQuantizer, opticalflow_packet


def parse_sample(line):
    if not line.startswith("BF_CORE_SAMPLE "):
        return None

    values = {}
    for part in line.split()[1:]:
        key, value = part.split("=", 1)
        values[key] = value

    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--axis-x", type=int, choices=(-1, 1), default=1
    )
    parser.add_argument(
        "--axis-y", type=int, choices=(-1, 1), default=1
    )
    args = parser.parse_args()

    quantizer = AngularMotionQuantizer()

    for line in sys.stdin:
        sample = parse_sample(line.strip())
        if sample is None:
            continue

        try:
            dt = float(sample["dt_s"])
            rate_x = float(sample["rate_x"]) * args.axis_x
            rate_y = float(sample["rate_y"]) * args.axis_y

            if not (
                math.isfinite(dt)
                and math.isfinite(rate_x)
                and math.isfinite(rate_y)
                and 0 < dt < 1
            ):
                quantizer.reset()
                continue

            motion_x, motion_y = quantizer.encode(
                rate_x, rate_y, dt
            )

            # Offline framing only.
            # Quality is illustrative, not validated for FC use.
            packet = opticalflow_packet(
                motion_x, motion_y, 255
            )

            print(
                "BF_MSP_PACKET"
                f" t_ns={sample['t_ns']}"
                f" motion_x={motion_x}"
                f" motion_y={motion_y}"
                f" hex={packet.hex()}"
            )

        except (ValueError, KeyError, OverflowError):
            quantizer.reset()
            continue


if __name__ == "__main__":
    main()
