#!/usr/bin/env python3
"""Receive optical-flow measurements from C++ over local UNIX IPC."""

import math
import os
import socket
import struct

SOCKET_PATH = "/tmp/jtzero_bf_flow.sock"
PACKET_FORMAT = "<BQdddB"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)


class BfFlowReceiver:
    def __init__(self, path=SOCKET_PATH):
        self.path = path

        if os.path.lexists(path):
            raise RuntimeError(
                f"IPC path already exists: {path}"
            )

        self.sock = socket.socket(
            socket.AF_UNIX,
            socket.SOCK_DGRAM,
        )
        self.sock.setblocking(False)

        try:
            self.sock.bind(path)
        except Exception:
            self.sock.close()
            raise

        self.identity = os.stat(path)

    def poll(self):
        samples = []

        while True:
            try:
                packet = self.sock.recv(128)
            except BlockingIOError:
                break

            if len(packet) != PACKET_SIZE:
                continue

            version, timestamp_ns, dt_s, rx, ry, quality = (
                struct.unpack(PACKET_FORMAT, packet)
            )

            if version != 1:
                continue

            if not all(math.isfinite(v) for v in (dt_s, rx, ry)):
                continue

            if dt_s <= 0 or dt_s > 0.2:
                continue

            samples.append((
                timestamp_ns,
                dt_s,
                rx,
                ry,
                quality,
            ))

        return samples

    def close(self):
        self.sock.close()

        try:
            current = os.stat(self.path)
            if (
                current.st_dev == self.identity.st_dev
                and current.st_ino == self.identity.st_ino
            ):
                os.unlink(self.path)
        except FileNotFoundError:
            pass
