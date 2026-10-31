#!/usr/bin/env python3
"""
JT-Zero: offline Betaflight MT timing model.

No UART, FC, camera or rangefinder access.

Models the conversion in Betaflight:
    flow_rate = motion / (200 * receiver_dt)

The first packet establishes a timestamp and is excluded
from velocity-error statistics.

This is a timing model, not a flight-ready adapter.
"""

import math
import struct

from msp_encoder import (
    AngularMotionQuantizer,
    decode_frame,
    opticalflow_packet,
)

RATE_X = 0.15
RATE_Y = -0.08
PERIOD_S = 0.020
COUNT = 100
SCALE = 200.0


def build_packets():
    quantizer = AngularMotionQuantizer()
    packets = []

    for i in range(COUNT):
        x, y = quantizer.encode(
            RATE_X, RATE_Y, PERIOD_S
        )

        frame = opticalflow_packet(x, y, 255)

        _, command, payload = decode_frame(frame)

        assert command == 0x1F02

        quality, mx, my = struct.unpack(
            "<Bii", payload
        )

        assert quality == 255

        packets.append({
            "index": i,
            "measurement_end": (i + 1) * PERIOD_S,
            "motion_x": mx,
            "motion_y": my,
        })

    return packets


def evaluate(name, packets, receive_times):
    if len(packets) != len(receive_times):
        raise ValueError("Packet/time count mismatch")

    reconstructed_x = 0.0
    reconstructed_y = 0.0
    evaluated_time = 0.0
    previous_rx = None
    invalid_dt = 0

    for packet, rx in zip(packets, receive_times):
        if previous_rx is None:
            previous_rx = rx
            continue

        dt = rx - previous_rx
        previous_rx = rx

        if not (0.0 < dt < 1.0):
            invalid_dt += 1
            continue

        # Betaflight MT receive-side calculation.
        flow_x = packet["motion_x"] / SCALE / dt
        flow_y = packet["motion_y"] / SCALE / dt

        # Reintegrate reconstructed rates.
        reconstructed_x += flow_x * dt
        reconstructed_y += flow_y * dt
        evaluated_time += dt

    expected_x = RATE_X * evaluated_time
    expected_y = RATE_Y * evaluated_time

    mean_x = (
        reconstructed_x / evaluated_time
        if evaluated_time else 0.0
    )

    mean_y = (
        reconstructed_y / evaluated_time
        if evaluated_time else 0.0
    )

    error_x = mean_x - RATE_X
    error_y = mean_y - RATE_Y

    print()
    print("SCENARIO:", name)
    print("Packets:", len(packets))
    print("Evaluated time:", round(evaluated_time, 6))
    print("Invalid receiver dt:", invalid_dt)
    print("Expected X:", round(expected_x, 6))
    print("Reconstructed X:", round(reconstructed_x, 6))
    print("Expected Y:", round(expected_y, 6))
    print("Reconstructed Y:", round(reconstructed_y, 6))
    print("Mean rate X:", round(mean_x, 6))
    print("Mean rate Y:", round(mean_y, 6))
    print("Rate error X:", round(error_x, 6))
    print("Rate error Y:", round(error_y, 6))

    return {
        "error_x": error_x,
        "error_y": error_y,
        "mean_x": mean_x,
        "mean_y": mean_y,
        "invalid_dt": invalid_dt,
    }


def main():
    packets = build_packets()

    # Case 1: regular reception, 20 ms.
    regular = [
        p["measurement_end"]
        for p in packets
    ]

    a = evaluate(
        "REGULAR_20MS",
        packets,
        regular,
    )

    # Case 2: deterministic receive-time jitter.
    # Measurement times remain regular.
    jittered = [
        p["measurement_end"]
        + 0.004 * math.sin(i * 0.7)
        for i, p in enumerate(packets)
    ]

    b = evaluate(
        "RECEIVE_JITTER",
        packets,
        jittered,
    )

    # Case 3: one 100 ms receiver delay.
    # Delayed packets remain ordered.
    delayed = list(regular)

    for i in range(40, 45):
        delayed[i] += 0.080

    # Ensure timestamps are strictly increasing
    # for the model, without reordering packets.
    for i in range(1, len(delayed)):
        if delayed[i] <= delayed[i - 1]:
            delayed[i] = delayed[i - 1] + 0.001

    c = evaluate(
        "RECEIVER_DELAY",
        packets,
        delayed,
    )

    # Case 4: missing packets.
    # No invented replacement measurements.
    selected = [
        i for i in range(COUNT)
        if not (40 <= i < 45)
    ]

    dropped_packets = [packets[i] for i in selected]
    dropped_times = [regular[i] for i in selected]

    d = evaluate(
        "FIVE_DROPPED_PACKETS",
        dropped_packets,
        dropped_times,
    )

    print()
    print("===== CHECKS =====")

    assert abs(a["error_x"]) < 0.005
    assert abs(a["error_y"]) < 0.005
    print("PASS: regular timing, mean-rate accuracy")

    assert b["invalid_dt"] == 0
    print("PASS: jitter scenario processed")

    assert c["invalid_dt"] == 0
    print("PASS: receiver-delay scenario processed")

    assert d["invalid_dt"] == 0
    print("PASS: dropped-packet scenario processed")

    print()
    print("PASS: Offline FC timing model completed")
    print("NOTE: Scenario errors are diagnostic,")
    print("not proof of flight compatibility.")
    print("UART: DISABLED")


if __name__ == "__main__":
    main()
