#!/usr/bin/env python3

"""JT-Zero MSPv2 offline sensor simulator.

No camera.
No rangefinder.
No UART.
No flight controller communication.
"""

import argparse
import math
import struct

from msp_encoder import (
    AngularMotionQuantizer,
    decode_frame,
    opticalflow_packet,
    rangefinder_packet,
)

PERIOD_S = 0.020
SAMPLES = 100
SYNTHETIC_RANGE_M = 1.50

# Artificial body-frame angular flow in rad/s.
SYNTHETIC_FLOW_X = 0.15
SYNTHETIC_FLOW_Y = -0.08


def decode_range(frame):
    _, command, payload = decode_frame(frame)
    assert command == 0x1F01
    return struct.unpack("<Bi", payload)


def decode_flow(frame):
    _, command, payload = decode_frame(frame)
    assert command == 0x1F02
    return struct.unpack("<Bii", payload)


def run():
    quantizer = AngularMotionQuantizer()

    total_x = 0
    total_y = 0
    flow_frames = 0
    range_frames = 0

    print("===== JT-ZERO MSPv2 OFFLINE SIMULATOR =====")
    print("UART: DISABLED")
    print("CAMERA: NOT REQUIRED")
    print("RANGEFINDER: NOT REQUIRED")
    print("FC TRANSMISSION: DISABLED")
    print()

    print("step,time_s,quality,motion_x,motion_y,range_mm")

    for step in range(SAMPLES):
        t = step * PERIOD_S

        # Simulated sensor dropout from 0.8 to 1.0 seconds.
        valid = not (40 <= step < 50)

        if not valid:
            quantizer.reset()
            print(f"{step},{t:.3f},0,NA,NA,NA")
            continue

        motion_x, motion_y = quantizer.encode(
            SYNTHETIC_FLOW_X,
            SYNTHETIC_FLOW_Y,
            PERIOD_S,
        )

        flow_frame = opticalflow_packet(
            motion_x,
            motion_y,
            quality=255,
        )

        range_frame = rangefinder_packet(
            SYNTHETIC_RANGE_M,
            quality=255,
        )

        quality, decoded_x, decoded_y = decode_flow(flow_frame)
        range_quality, range_mm = decode_range(range_frame)

        assert quality == 255
        assert range_quality == 255
        assert range_mm == 1500

        total_x += decoded_x
        total_y += decoded_y

        flow_frames += 1
        range_frames += 1

        print(
            f"{step},{t:.3f},{quality},"
            f"{decoded_x},{decoded_y},{range_mm}"
        )

    expected_x = SYNTHETIC_FLOW_X * PERIOD_S * 200 * flow_frames
    expected_y = SYNTHETIC_FLOW_Y * PERIOD_S * 200 * flow_frames

    print()
    print("===== SUMMARY =====")
    print(f"Flow frames: {flow_frames}")
    print(f"Range frames: {range_frames}")
    print(f"Total motion X: {total_x}")
    print(f"Total motion Y: {total_y}")
    print(f"Ideal motion X: {expected_x:.3f}")
    print(f"Ideal motion Y: {expected_y:.3f}")

    # Two valid segments, each with independently reset quantization.
    # Error per segment must remain below one integer count.
    assert abs(total_x - expected_x) <= 2.0
    assert abs(total_y - expected_y) <= 2.0
    assert flow_frames == 90
    assert range_frames == 90

    print()
    print("PASS: MSPv2 offline simulation")
    print("PASS: Frame encoding/decoding")
    print("PASS: Fractional motion accumulation")
    print("PASS: Dropout handling")
    print()
    print("NOT VERIFIED: Betaflight gyro/axis compatibility")
    print("NOT VERIFIED: Real UART timing")
    print("NOT VERIFIED: Real sensor measurements")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()

    if args.run:
        run()
    else:
        parser.print_help()
