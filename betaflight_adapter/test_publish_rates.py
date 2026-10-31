#!/usr/bin/env python3
"""
JT-Zero: offline MSPv2 publication-rate comparison.

Sensor simulation: 100 Hz.
Publication: 20 Hz or 50 Hz.

No UART, camera, rangefinder or FC access.
"""

import math
import statistics

from msp_encoder import AngularMotionQuantizer

SENSOR_HZ = 100
DURATION_S = 10
RATE_X = 0.15
RATE_Y = -0.08
SCALE = 200.0


def percentile(values, p):
    data = sorted(values)
    if not data:
        return 0.0
    index = math.ceil(len(data) * p / 100) - 1
    return data[max(0, min(index, len(data) - 1))]


def build_packets(publish_hz):
    assert SENSOR_HZ % publish_hz == 0

    samples_per_packet = SENSOR_HZ // publish_hz
    sample_dt = 1.0 / SENSOR_HZ
    quantizer = AngularMotionQuantizer()

    packets = []
    sum_x = 0.0
    sum_y = 0.0

    total_samples = SENSOR_HZ * DURATION_S

    for i in range(total_samples):
        sum_x += RATE_X * sample_dt
        sum_y += RATE_Y * sample_dt

        if (i + 1) % samples_per_packet:
            continue

        packet_dt = samples_per_packet * sample_dt

        mx, my = quantizer.encode(
            sum_x / packet_dt,
            sum_y / packet_dt,
            packet_dt,
        )

        packets.append({
            "index": len(packets),
            "time": (i + 1) * sample_dt,
            "mx": mx,
            "my": my,
        })

        sum_x = 0.0
        sum_y = 0.0

    return packets


def analyze(name, packets, receive_times, rate_hz):
    if len(packets) != len(receive_times):
        raise ValueError("Length mismatch")

    errors_x = []
    errors_y = []
    intervals = []
    estimates_x = []
    estimates_y = []

    total_x = 0.0
    total_y = 0.0
    previous_rx = None

    for packet, rx in zip(packets, receive_times):
        if previous_rx is None:
            previous_rx = rx
            continue

        dt = rx - previous_rx
        previous_rx = rx

        if dt <= 0:
            raise ValueError("Non-monotonic receive timestamps")

        vx = packet["mx"] / (SCALE * dt)
        vy = packet["my"] / (SCALE * dt)

        estimates_x.append(vx)
        estimates_y.append(vy)

        errors_x.append(abs(vx - RATE_X))
        errors_y.append(abs(vy - RATE_Y))
        intervals.append(dt)

        total_x += packet["mx"] / SCALE
        total_y += packet["my"] / SCALE

    result = {
        "name": name,
        "hz": rate_hz,
        "packets": len(packets),
        "min_dt_ms": min(intervals) * 1000,
        "max_dt_ms": max(intervals) * 1000,
        "p95_x": percentile(errors_x, 95),
        "p95_y": percentile(errors_y, 95),
        "max_x": max(errors_x),
        "max_y": max(errors_y),
        "mean_x": statistics.mean(estimates_x),
        "mean_y": statistics.mean(estimates_y),
        "total_x": total_x,
        "total_y": total_y,
    }

    print()
    print("SCENARIO:", name)
    print("Publication rate:", rate_hz, "Hz")
    print("Packets:", result["packets"])
    print(
        "Receive dt min/max:",
        f'{result["min_dt_ms"]:.3f}',
        f'{result["max_dt_ms"]:.3f}',
        "ms",
    )
    print("Mean X:", f'{result["mean_x"]:.6f}')
    print("Mean Y:", f'{result["mean_y"]:.6f}')
    print("P95 error X:", f'{result["p95_x"]:.6f}')
    print("P95 error Y:", f'{result["p95_y"]:.6f}')
    print("Max error X:", f'{result["max_x"]:.6f}')
    print("Max error Y:", f'{result["max_y"]:.6f}')
    print("Integrated X:", f'{result["total_x"]:.6f}')
    print("Integrated Y:", f'{result["total_y"]:.6f}')

    return result


def run_rate(hz):
    packets = build_packets(hz)
    period = 1.0 / hz

    regular = [p["time"] for p in packets]

    baseline = analyze(
        f"{hz}HZ_REGULAR",
        packets,
        regular,
        hz,
    )

    jitter_amplitude = 0.004

    jittered = [
        p["time"] + jitter_amplitude * math.sin(i * 0.7)
        for i, p in enumerate(packets)
    ]

    jitter = analyze(
        f"{hz}HZ_JITTER",
        packets,
        jittered,
        hz,
    )

    # Artificial delivery backlog:
    # delay five packets, preserving their order.
    delayed = list(regular)

    for i in range(40, 45):
        delayed[i] += 4 * period

    for i in range(1, len(delayed)):
        if delayed[i] <= delayed[i - 1]:
            delayed[i] = delayed[i - 1] + 0.001

    burst = analyze(
        f"{hz}HZ_BURST",
        packets,
        delayed,
        hz,
    )

    # Drop five packets without retransmission.
    selected = [
        i for i in range(len(packets))
        if not 40 <= i < 45
    ]

    dropped_packets = [packets[i] for i in selected]
    dropped_times = [regular[i] for i in selected]

    dropout = analyze(
        f"{hz}HZ_DROPOUT",
        dropped_packets,
        dropped_times,
        hz,
    )

    assert baseline["packets"] == hz * DURATION_S
    assert jitter["min_dt_ms"] > 0
    assert burst["min_dt_ms"] > 0
    assert dropout["packets"] == hz * DURATION_S - 5

    return {
        "regular": baseline,
        "jitter": jitter,
        "burst": burst,
        "dropout": dropout,
    }


def main():
    print("===== JT-ZERO MSPv2 RATE COMPARISON =====")
    print("Sensor simulation:", SENSOR_HZ, "Hz")
    print("Duration:", DURATION_S, "seconds")
    print("UART: DISABLED")
    print("FC: NOT CONNECTED")

    results = {}

    for hz in (20, 50):
        results[hz] = run_rate(hz)

    print()
    print("===== COMPARISON =====")

    for scenario in ("regular", "jitter", "burst", "dropout"):
        a = results[20][scenario]
        b = results[50][scenario]

        print()
        print(scenario.upper())
        print(
            "20Hz P95 X:",
            f'{a["p95_x"]:.6f}',
            "50Hz P95 X:",
            f'{b["p95_x"]:.6f}',
        )
        print(
            "20Hz MAX X:",
            f'{a["max_x"]:.6f}',
            "50Hz MAX X:",
            f'{b["max_x"]:.6f}',
        )

    print()
    print("PASS: Publication-rate comparison completed")
    print("NOT VERIFIED: FC filtering and gyro compensation")
    print("NOT VERIFIED: Real transport scheduling")


if __name__ == "__main__":
    main()
