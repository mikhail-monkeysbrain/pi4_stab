#!/usr/bin/env python3
"""Local IPC publisher for Betaflight sensor samples."""

import socket
import struct

SOCKET_PATH = "/tmp/jtzero_bf_imu.sock"

PACKET_VERSION = 1
TYPE_IMU = 102
TYPE_ATTITUDE = 108

PACKET_FORMAT = "<BBQ9f"


class BfImuPublisher:
    def __init__(self, path=SOCKET_PATH):
        self.sock = socket.socket(
            socket.AF_UNIX,
            socket.SOCK_DGRAM,
        )
        self.sock.setblocking(False)
        self.path = path
        self.last_error = None

    def publish(self, sample):
        values = tuple(float(v) for v in sample.values)

        if sample.command == TYPE_IMU:
            if len(values) != 9:
                return False

        elif sample.command == TYPE_ATTITUDE:
            if len(values) != 3:
                return False
            values += (0.0,) * 6

        else:
            return False

        packet = struct.pack(
            PACKET_FORMAT,
            PACKET_VERSION,
            sample.command,
            sample.timestamp_ns,
            *values,
        )

        try:
            self.sock.sendto(packet, self.path)
            self.last_error = None
            return True
        except OSError as exc:
            self.last_error = exc.errno
            return False

    def close(self):
        self.sock.close()
