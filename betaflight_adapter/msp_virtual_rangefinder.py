#!/usr/bin/env python3
"""JT-Zero: Betaflight altitude -> virtual MSP rangefinder."""

import argparse
import math
import time

import serial

from msp_altitude_receiver import (
    MspAltitudeParser,
    MSP_REQUEST,
)
from msp_altitude_reference import MspAltitudeReference
from msp_encoder import rangefinder_packet


REQUEST_PERIOD_NS = 20_000_000
RESPONSE_TIMEOUT_NS = 150_000_000

RANGE_MIN_M = 0.05
RANGE_MAX_M = 100.0


def write_frame(uart, frame):
    written = uart.write(frame)

    if written != len(frame):
        raise IOError(
            f"Incomplete MSP write: {written}/{len(frame)}"
        )


def run(port, baud):
    reference = MspAltitudeReference(
        initial_height_m=0.10,
        calibration_s=5.0,
        max_age_s=0.15,
    )

    parser = MspAltitudeParser()

    print(f"JT-Zero virtual rangefinder: {port} @ {baud}")
    print("Source: MSP_ALTITUDE")
    print("Output: MSP2_SENSOR_RANGEFINDER_LIDARMT")
    print("Calibration: 5 seconds -> 0.10 m")
    print("Optical Flow: untouched", flush=True)

    with serial.Serial(
        port,
        baud,
        timeout=0.005,
        write_timeout=0.1,
    ) as uart:

        next_request_ns = time.monotonic_ns()
        pending = False
        request_ns = 0
        last_sample_ns = None
        last_print_ns = 0

        while True:
            now_ns = time.monotonic_ns()

            if pending and now_ns - request_ns > RESPONSE_TIMEOUT_NS:
                pending = False
                parser = MspAltitudeParser()

                # Do not silently redefine ground level after
                # an established reference is lost.
                if reference.reference_m is None:
                    reference.reset()

                print("MSP TIMEOUT", flush=True)

            if not pending and now_ns >= next_request_ns:
                write_frame(uart, MSP_REQUEST)

                request_ns = time.monotonic_ns()
                next_request_ns = request_ns + REQUEST_PERIOD_NS
                pending = True

            data = uart.read(256)

            if not data:
                continue

            for altitude_m in parser.feed(data):
                if not pending:
                    continue

                sample_ns = time.monotonic_ns()
                pending = False

                if (
                    last_sample_ns is not None
                    and sample_ns <= last_sample_ns
                ):
                    continue

                last_sample_ns = sample_ns

                # Preserve the calibrated ground reference across
                # communication gaps. Never re-zero during operation.
                if (
                    reference.reference_m is not None
                    and reference.last_ns is not None
                    and sample_ns - reference.last_ns
                    > reference.max_age_ns
                ):
                    reference.last_ns = None

                result = reference.update(
                    altitude_m=altitude_m,
                    sample_ns=sample_ns,
                    now_ns=sample_ns,
                )

                if not result.ready:
                    if sample_ns - last_print_ns >= 1_000_000_000:
                        print(
                            f"STATE={result.reason} "
                            f"elapsed={result.elapsed_s:.1f}s",
                            flush=True,
                        )
                        last_print_ns = sample_ns
                    continue

                height_m = result.height_m

                if (
                    not math.isfinite(height_m)
                    or not RANGE_MIN_M <= height_m <= RANGE_MAX_M
                ):
                    print(
                        f"RANGE INVALID: {height_m}",
                        flush=True,
                    )
                    continue

                write_frame(
                    uart,
                    rangefinder_packet(height_m, quality=255),
                )

                if sample_ns - last_print_ns >= 1_000_000_000:
                    print(
                        f"BARO={altitude_m:.3f}m "
                        f"REF={result.reference_m:.3f}m "
                        f"RANGE_TX={height_m:.3f}m",
                        flush=True,
                    )
                    last_print_ns = sample_ns


if __name__ == "__main__":
    cli = argparse.ArgumentParser()
    cli.add_argument("--port", default="/dev/ttyAMA4")
    cli.add_argument("--baud", type=int, default=115200)
    args = cli.parse_args()

    try:
        run(args.port, args.baud)
    except KeyboardInterrupt:
        print("\nStopped")
