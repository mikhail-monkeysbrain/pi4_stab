#!/usr/bin/env python3
"""Read Betaflight MSP_ALTITUDE and calculate OF height.

Read-only MSP requests.
Does not transmit Optical Flow.
Does not access cameras or ArduPilot.
"""

import argparse
import struct
import time

import serial

from msp_altitude_reference import MspAltitudeReference


MSP_ALTITUDE = 109
MSP_REQUEST = b"$M<\x00\x6d\x6d"

REQUEST_PERIOD_NS = 20_000_000
RESPONSE_TIMEOUT_NS = 150_000_000


class MspAltitudeParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        results = []

        while len(self.buffer) >= 6:
            start = self.buffer.find(b"$M")

            if start < 0:
                self.buffer[:] = self.buffer[-1:]
                break

            if start:
                del self.buffer[:start]

            if len(self.buffer) < 6:
                break

            if self.buffer[2] not in (ord(">"), ord("!")):
                del self.buffer[0]
                continue

            size = self.buffer[3]
            total = size + 6

            if len(self.buffer) < total:
                break

            frame = bytes(self.buffer[:total])
            del self.buffer[:total]

            crc = 0
            for value in frame[3:-1]:
                crc ^= value

            if crc != frame[-1]:
                continue

            if frame[2] != ord(">"):
                continue

            if frame[4] != MSP_ALTITUDE:
                continue

            if size < 4:
                continue

            altitude_cm = struct.unpack_from(
                "<i", frame, 5
            )[0]

            results.append(altitude_cm / 100.0)

        return results


def run(port, baud):
    reference = MspAltitudeReference(
        initial_height_m=0.10,
        calibration_s=5.0,
        max_age_s=0.15,
    )

    parser = MspAltitudeParser()

    print(
        f"MSP ALTITUDE: {port} @ {baud}",
        flush=True,
    )

    print(
        "OF output disabled. Altitude monitoring only.",
        flush=True,
    )

    while True:
        try:
            with serial.Serial(
                port,
                baud,
                timeout=0.005,
                write_timeout=0.1,
            ) as uart:

                print("UART CONNECTED", flush=True)

                reference.reset()
                parser = MspAltitudeParser()

                pending = False
                request_ns = 0
                next_request_ns = time.monotonic_ns()

                last_sample_ns = None
                last_print_ns = 0

                while True:
                    now_ns = time.monotonic_ns()

                    if pending:
                        if now_ns - request_ns > RESPONSE_TIMEOUT_NS:
                            pending = False
                            reference.reset()
                            parser = MspAltitudeParser()
                            print(
                                "MSP TIMEOUT: reference reset",
                                flush=True,
                            )

                    if not pending and now_ns >= next_request_ns:
                        uart.write(MSP_REQUEST)

                        request_ns = time.monotonic_ns()
                        next_request_ns = (
                            request_ns + REQUEST_PERIOD_NS
                        )
                        pending = True

                    data = uart.read(256)

                    if not data:
                        continue

                    heights = parser.feed(data)

                    for altitude_m in heights:
                        if not pending:
                            continue

                        sample_ns = time.monotonic_ns()

                        if (
                            last_sample_ns is not None
                            and sample_ns <= last_sample_ns
                        ):
                            continue

                        last_sample_ns = sample_ns
                        pending = False

                        result = reference.update(
                            altitude_m=altitude_m,
                            sample_ns=sample_ns,
                            now_ns=sample_ns,
                        )

                        if (
                            sample_ns - last_print_ns
                            < 200_000_000
                            and result.reason != "READY"
                        ):
                            continue

                        if (
                            sample_ns - last_print_ns
                            < 200_000_000
                        ):
                            continue

                        last_print_ns = sample_ns

                        if result.ready:
                            print(
                                f"READY "
                                f"MSP={altitude_m:.3f}m "
                                f"REF={result.reference_m:.3f}m "
                                f"OF_HEIGHT={result.height_m:.3f}m",
                                flush=True,
                            )
                        else:
                            print(
                                f"{result.reason} "
                                f"MSP={altitude_m:.3f}m "
                                f"ELAPSED={result.elapsed_s:.2f}s",
                                flush=True,
                            )

        except (serial.SerialException, OSError) as exc:
            reference.reset()

            print(
                f"UART ERROR: {exc}",
                flush=True,
            )

            time.sleep(1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()

    ap.add_argument(
        "--serial",
        default="/dev/ttyAMA4",
    )

    ap.add_argument(
        "--baud",
        type=int,
        default=115200,
    )

    args = ap.parse_args()

    try:
        run(args.serial, args.baud)
    except KeyboardInterrupt:
        print("\nSTOPPED")
