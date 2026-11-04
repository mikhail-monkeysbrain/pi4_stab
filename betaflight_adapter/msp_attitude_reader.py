#!/usr/bin/env python3
"""Betaflight MSP_ATTITUDE parser."""

import math
import struct
import time
from dataclasses import dataclass


MSP_ATTITUDE = 108
MSP_ATTITUDE_REQUEST = b"$M<\x00\x6c\x6c"


@dataclass(frozen=True)
class AttitudeSample:
    timestamp_ns: int
    roll: float
    pitch: float
    yaw: float


class MspAttitudeReader:
    def __init__(self, uart):
        self.uart = uart
        self.buffer = bytearray()

    def request(self):
        self.uart.write(MSP_ATTITUDE_REQUEST)

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

            if command != MSP_ATTITUDE or size != 6:
                continue

            roll, pitch, yaw = struct.unpack(
                "<hhh", frame[5:-1]
            )

            return AttitudeSample(
                timestamp_ns=time.monotonic_ns(),
                roll=math.radians(roll / 10.0),
                pitch=math.radians(pitch / 10.0),
                yaw=math.radians(yaw),
            )

        return None
