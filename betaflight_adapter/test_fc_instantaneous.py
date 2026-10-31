#!/usr/bin/env python3
"""
JT-Zero / Betaflight MT instantaneous-rate diagnostics.

OFFLINE ONLY. No UART, FC, camera or rangefinder.

Assumed Betaflight conversion:
    flow_rate = motion_counts / (200 * receive_dt)

This model excludes gyro subtraction, orientation,
filtering and FC scheduling effects.
"""

import math
import statistics

from msp_encoder import AngularMotionQuantizer

RATE_X = 0.15
RATE_Y = -0.08
SCALE = 200.0
COUNT = 200


def percentile(values, percent):
    values = sorted(values)
    if not values:
        return 0.0
    index = math.ceil(len(values) * percent / 100) - 1
    return values[max(0, min(index, len(values) - 1))]


def build_samples(hz):
    dt = 1.0 / hz
    quantizer = AngularMotionQuantizer()
    samples = []

    for i in range(COUNT):
        mx, my = quantizer.encode(RATE_X, RATE_Y, dt)

        samples.append({
            "index": i,
            "time": (i + 1) * dt,
            "mx": mx,
            "my": my,
        })

    return samples


def simulate(name, samples, receive_times):
    assert len(samples) == len(receive_times)

    errors_x = []
    errors_y = []
    intervals = []
    estimates_x = []
    estimates_y = []

    previous_rx = None

    for sample, rx in zip(samples, receive_times):
        if previous_rx is None:
            previous_rx = rx
            continue

        dt = rx - previous_rx
        previous_rx = rx

        if dt <= 0:
            raise ValueError(
                f"{name}: non-monotonic receive time"
            )

        vx = sample["mx"] / (SCALE * dt)
        vy = sample["my"] / (SCALE * dt)

        estimates_x.append(vx)
        estimates_y.append(vy)

        errors_x.append(abs(vx - RATE_X))
        errors_y.append(abs(vy - RATE_Y))
        intervals.append(dt)

    result = {
        "name": name,
        "count": len(intervals),
        "min_dt_ms": min(intervals) * 1000,
        "max_dt_ms": max(intervals) * 1000,
        "mean_x": statistics.mean(estimates_x),
        "mean_y": statistics.mean(estimates_y),
        "p95_x": percentile(errors_x, 95),
        "p95_y": percentile(errors_y, 95),
        "max_x": max(errors_x),
        "max_y": max(errors_y),
    }

    print()
    print("SCENARIO:", name)
    print("Evaluated packets:", result["count"])
    print(
        "Receive dt min/max ms:",
        f'{result["min_dt_ms"]:.3f}',
        f'{result["max_dt_ms"]:.3f}',
    )
    print("Mean X:", f'{result["mean_x"]:.6f}')
    print("Mean Y:", f'{result["mean_y"]:.6f}')
    print("P95 absolute error X:", f'{result["p95_x"]:.6f}')
    print("P95 absolute error Y:", f'{result["p95_y"]:.6f}')
    print("Max absolute error X:", f'{result["max_x"]:.6f}')
    print("Max absolute error Y:", f'{result["max_y"]:.6f}')

    return result


def main():
    print("===== JT-ZERO INSTANTANEOUS FLOW =====")
    print("MODE: OFFLINE")
    print("UART: DISABLED")
    print("FC: NOT CONNECTED")
    print("Flow units: rad/s")
    print()

    samples = build_samples(50)
    regular = [s["time"] for s in samples]

    baseline = simulate(
        "REGULAR_50HZ",
        samples,
        regular,
    )

    jitter = [
        s["time"] + 0.004 * math.sin(i * 0.7)
        for i, s in enumerate(samples)
    ]

    jitter_result = simulate(
        "JITTER_4MS",
        samples,
        jitter,
    )

    # Five consecutive packets experience a delayed
    # delivery. Preserve order and model burst arrival
    # using a 1 ms minimum separation.
    delayed = list(regular)

    for i in range(40, 45):
        delayed[i] += 0.080

    for i in range(1, len(delayed)):
        if delayed[i] <= delayed[i - 1]:
            delayed[i] = delayed[i - 1] + 0.001

    delay_result = simulate(
        "DELAYED_BURST",
        samples,
        delayed,
    )

    # Drop five packets without inventing replacement data.
    selected = [
        i for i in range(COUNT)
        if not 40 <= i < 45
    ]

    dropped = [samples[i] for i in selected]
    dropped_rx = [regular[i] for i in selected]

    dropout_result = simulate(
        "FIVE_DROPPED_PACKETS",
        dropped,
        dropped_rx,
    )

    print()
    print("===== FREQUENCY COMPARISON =====")

    for hz in (20, 50, 100):
        data = build_samples(hz)
        times = [s["time"] for s in data]

        result = simulate(
            f"REGULAR_{hz}HZ",
            data,
            times,
        )

        print(
            f"Quantization step at {hz} Hz:",
            f"{hz / SCALE:.3f} rad/s"
        )

    print()
    print("===== ASSERTIONS =====")

    assert baseline["count"] == COUNT - 1
    assert abs(baseline["mean_x"] - RATE_X) < 0.01
    assert abs(baseline["mean_y"] - RATE_Y) < 0.01

    assert jitter_result["min_dt_ms"] > 0
    assert delay_result["min_dt_ms"] > 0

    assert dropout_result["count"] == COUNT - 6
    assert dropout_result["max_dt_ms"] > 100

    print("PASS: Regular timing")
    print("PASS: Receive jitter")
    print("PASS: Delayed burst")
    print("PASS: Dropped packets")
    print("PASS: Frequency comparison")

    print()
    print("NOT VERIFIED: FC filtering")
    print("NOT VERIFIED: FC gyro subtraction")
    print("NOT VERIFIED: Real UART scheduling")
    print("NOT VERIFIED: Sensor axis compatibility")


if __name__ == "__main__":
    main()
