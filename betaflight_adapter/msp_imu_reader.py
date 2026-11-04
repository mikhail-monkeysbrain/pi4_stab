#!/usr/bin/env python3
"""MSP v1 RAW_IMU reader for Betaflight."""

import struct
import time
from dataclasses import dataclass


MSP_RAW_IMU = 102
MSP_RAW_IMU_REQUEST = b"$M<\x00\x66\x66"


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    acc: tuple
    gyro: tuple
    mag: tuple


class MspImuReader:
    def __init__(self, uart):
        self.uart = uart
        self.buffer = bytearray()

    def request(self):
        self.uart.write(MSP_RAW_IMU_REQUEST)

    def poll(self):
        available = self.uart.in_waiting

        if available:
            self.buffer.extend(self.uart.read(available))

        if len(self.buffer) > 4096:
            self.buffer = self.buffer[-4096:]

        while len(self.buffer) >= 6:
            pos = self.buffer.find(b"$M>")

            if pos < 0:
                self.buffer.clear()
                return None

            if pos:
                del self.buffer[:pos]

            if len(self.buffer) < 6:
                return None

            size = self.buffer[3]
            command = self.buffer[4]
            total = 6 + size

            if len(self.buffer) < total:
                return None

            frame = bytes(self.buffer[:total])
            del self.buffer[:total]

            checksum = size ^ command

            for value in frame[5:-1]:
                checksum ^= value

            if checksum != frame[-1]:
                continue

            if command != MSP_RAW_IMU or size != 18:
                continue

            values = struct.unpack("<9h", frame[5:-1])

            return ImuSample(
                timestamp_ns=time.monotonic_ns(),
                acc=values[0:3],
                gyro=values[3:6],
                mag=values[6:9],
            )

        return None
