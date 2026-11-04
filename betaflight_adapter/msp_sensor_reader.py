#!/usr/bin/env python3
"""Unified Betaflight MSP sensor reader."""

import math
import struct
import time
from dataclasses import dataclass

MSP_RAW_IMU = 102
MSP_ATTITUDE = 108


@dataclass(frozen=True)
class SensorSample:
    command: int
    timestamp_ns: int
    values: tuple


class MspSensorReader:
    def __init__(self, uart):
        self.uart = uart
        self.buffer = bytearray()

    def request(self, command):
        if command not in (MSP_RAW_IMU, MSP_ATTITUDE):
            raise ValueError("Unsupported MSP command")

        checksum = command
        packet = bytes(
            (36, 77, 60, 0, command, checksum)
        )
        written = self.uart.write(packet)

        if written != len(packet):
            raise IOError("Incomplete MSP request")

    def poll(self):
        available = self.uart.in_waiting

        if available:
            self.buffer.extend(
                self.uart.read(min(available, 512))
            )

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
            total = size + 6

            if len(self.buffer) < total:
                return None

            frame = bytes(self.buffer[:total])
            del self.buffer[:total]

            checksum = size ^ command

            for value in frame[5:-1]:
                checksum ^= value

            if checksum != frame[-1]:
                continue

            timestamp_ns = time.monotonic_ns()
            payload = frame[5:-1]

            if command == MSP_RAW_IMU and size == 18:
                values = struct.unpack("<9h", payload)

                return SensorSample(
                    command,
                    timestamp_ns,
                    values,
                )

            if command == MSP_ATTITUDE and size == 6:
                roll, pitch, yaw = struct.unpack(
                    "<hhh", payload
                )

                return SensorSample(
                    command,
                    timestamp_ns,
                    (
                        math.radians(roll / 10.0),
                        math.radians(pitch / 10.0),
                        math.radians(yaw),
                    ),
                )

        return None
