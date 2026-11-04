#!/usr/bin/env python3
"""JT-Zero Betaflight: TF-Luna TX + MSP RAW_IMU RX."""

import argparse
import time

import serial

from msp_encoder import rangefinder_packet
from msp_imu_reader import MspImuReader


LUNA_MIN_CM = 5
LUNA_MAX_CM = 800

IMU_PERIOD_NS = 10_000_000
IMU_TIMEOUT_NS = 30_000_000


class TfLunaParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)

        if len(self.buffer) > 512:
            self.buffer = self.buffer[-512:]

        results = []

        while len(self.buffer) >= 9:
            if self.buffer[:2] != b"\x59\x59":
                del self.buffer[0]
                continue

            frame = bytes(self.buffer[:9])

            if (sum(frame[:8]) & 0xFF) != frame[8]:
                del self.buffer[0]
                continue

            del self.buffer[:9]

            distance_cm = frame[2] | (frame[3] << 8)
            strength = frame[4] | (frame[5] << 8)

            if not LUNA_MIN_CM <= distance_cm <= LUNA_MAX_CM:
                continue

            if strength in (0, 65535):
                continue

            results.append((distance_cm / 100.0, strength))

        return results


def run(luna_port, msp_port, baud):
    parser = TfLunaParser()

    range_tx = 0
    imu_rx = 0
    imu_timeout = 0

    pending_imu = False
    request_ns = 0
    next_imu_ns = time.monotonic_ns()
    last_report_ns = next_imu_ns

    print("JT-Zero Betaflight MSP runtime")
    print(f"TF-Luna: {luna_port}")
    print(f"MSP FC:  {msp_port}")
    print(f"Baud:    {baud}")
    print("IMU request: 100 Hz target")
    print("Optical Flow: not connected yet", flush=True)

    with serial.Serial(
        luna_port,
        115200,
        timeout=0,
    ) as luna, serial.Serial(
        msp_port,
        baud,
        timeout=0,
        write_timeout=0.1,
    ) as fc:

        imu = MspImuReader(fc)

        while True:
            now = time.monotonic_ns()

            if pending_imu and now - request_ns >= IMU_TIMEOUT_NS:
                pending_imu = False
                imu_timeout += 1
                imu.buffer.clear()
                fc.reset_input_buffer()

            if not pending_imu and now >= next_imu_ns:
                imu.request()
                request_ns = time.monotonic_ns()
                pending_imu = True
                next_imu_ns = request_ns + IMU_PERIOD_NS

            sample = imu.poll()

            if sample is not None and pending_imu:
                pending_imu = False
                imu_rx += 1

            available = luna.in_waiting

            if available:
                data = luna.read(min(available, 512))

                for distance_m, strength in parser.feed(data):
                    packet = rangefinder_packet(
                        distance_m,
                        quality=255,
                    )

                    written = fc.write(packet)

                    if written != len(packet):
                        raise IOError("Incomplete MSP rangefinder write")

                    range_tx += 1

            now = time.monotonic_ns()

            if now - last_report_ns >= 1_000_000_000:
                print(
                    f"IMU_RX={imu_rx} "
                    f"IMU_TIMEOUT={imu_timeout} "
                    f"RANGE_TX={range_tx}",
                    flush=True,
                )
                last_report_ns = now

            time.sleep(0.001)


if __name__ == "__main__":
    cli = argparse.ArgumentParser()
    cli.add_argument("--luna-port", default="/dev/serial0")
    cli.add_argument("--msp-port", default="/dev/ttyAMA4")
    cli.add_argument("--baud", type=int, default=115200)
    args = cli.parse_args()

    try:
        run(args.luna_port, args.msp_port, args.baud)
    except KeyboardInterrupt:
        print("\nStopped")
