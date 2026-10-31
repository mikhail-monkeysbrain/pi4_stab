#!/usr/bin/env python3
"""Offline-safe-by-default MSPv2 rangefinder feedback probe."""

import argparse
import csv
import os
import select
import struct
import termios
import time

from msp_encoder import rangefinder_packet

PORT = "/dev/ttyAMA4"
OUTPUT = "/tmp/jtzero_rangefinder_feedback.csv"
MSP_ALTITUDE = 109


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 255
    return crc


def request(command):
    body = struct.pack("<BHH", 0, command, 0)
    return b"$X<" + body + bytes([crc8(body)])


def configure(fd):
    a = termios.tcgetattr(fd)
    a[0] = 0
    a[1] = 0
    a[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    a[3] = 0
    a[4] = termios.B115200
    a[5] = termios.B115200
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)


class Reader:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []

        while True:
            pos = self.buffer.find(b"$X")

            if pos < 0:
                self.buffer.clear()
                break

            if pos:
                del self.buffer[:pos]

            if len(self.buffer) < 9:
                break

            if self.buffer[2] not in (ord(">"), ord("!")):
                del self.buffer[0]
                continue

            length = struct.unpack_from("<H", self.buffer, 6)[0]

            if length > 512:
                del self.buffer[0]
                continue

            total = 9 + length

            if len(self.buffer) < total:
                break

            frame = bytes(self.buffer[:total])
            del self.buffer[:total]

            if crc8(frame[3:-1]) != frame[-1]:
                self.crc_errors += 1
                continue

            command = struct.unpack_from("<H", frame, 4)[0]
            payload = frame[8:-1]
            error = frame[2] == ord("!")

            frames.append((command, payload, error))

        return frames


def write_all(fd, data):
    offset = 0

    while offset < len(data):
        _, writable, _ = select.select([], [fd], [], 0.2)

        if not writable:
            raise TimeoutError("UART write timeout")

        n = os.write(fd, data[offset:])

        if n <= 0:
            raise OSError("UART write failed")

        offset += n


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--transmit", action="store_true")
    args = parser.parse_args()

    print("PORT:", PORT)
    print("BAUD: 115200")
    print("MSP_ALTITUDE:", MSP_ALTITUDE)
    print("RANGEFINDER TX:", "ENABLED" if args.transmit else "DISABLED")
    print("CSV:", OUTPUT)

    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    reader = Reader()
    rows = []
    sent = 0
    requests = 0

    try:
        configure(fd)
        termios.tcflush(fd, termios.TCIFLUSH)

        # 1 s baseline, 2 s injection, 1 s recovery.
        duration = 4.0
        start = time.monotonic()

        next_query = start
        next_range = start + 1.0

        while True:
            now = time.monotonic()
            elapsed = now - start

            if elapsed >= duration:
                break

            if elapsed < 1.0:
                phase = "BEFORE"
            elif elapsed < 3.0:
                phase = "DURING"
            else:
                phase = "AFTER"

            if now >= next_query:
                write_all(fd, request(MSP_ALTITUDE))
                requests += 1
                next_query += 0.2

                if next_query < now:
                    next_query = now + 0.2

            if args.transmit and phase == "DURING":
                if now >= next_range and sent < 40:
                    write_all(fd, rangefinder_packet(1.5, 255))
                    sent += 1
                    next_range += 0.05

                    if next_range < now:
                        next_range = now + 0.05

            ready, _, _ = select.select([fd], [], [], 0.005)

            if not ready:
                continue

            data = os.read(fd, 4096)

            for command, payload, error in reader.feed(data):
                if command != MSP_ALTITUDE:
                    continue

                # Preserve raw payload; do not assume a layout.
                raw = payload.hex(" ")

                altitude_cm = None
                if not error and len(payload) >= 4:
                    altitude_cm = struct.unpack_from("<i", payload)[0]

                rows.append({
                    "elapsed_s": round(time.monotonic() - start, 4),
                    "phase": phase,
                    "command": command,
                    "error": int(error),
                    "payload_len": len(payload),
                    "payload_hex": raw,
                    "altitude_cm_candidate": altitude_cm,
                })

        with open(OUTPUT, "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "elapsed_s",
                    "phase",
                    "command",
                    "error",
                    "payload_len",
                    "payload_hex",
                    "altitude_cm_candidate",
                ],
            )
            writer.writeheader()
            writer.writerows(rows)

        print()
        print("===== RESULTS =====")
        print("Altitude requests:", requests)
        print("Altitude responses:", len(rows))
        print("Range packets sent:", sent)
        print("CRC errors:", reader.crc_errors)

        for phase in ("BEFORE", "DURING", "AFTER"):
            subset = [r for r in rows if r["phase"] == phase]
            print()
            print("PHASE:", phase)
            print("Responses:", len(subset))

            for r in subset[:5]:
                print(
                    r["elapsed_s"],
                    "cm_candidate=", r["altitude_cm_candidate"],
                    "payload=", r["payload_hex"],
                    "error=", r["error"],
                )

        print()
        print("CSV:", OUTPUT)
        print("NOTE: altitude is not raw rangefinder distance")

        if not rows:
            raise SystemExit("FAIL: no MSP_ALTITUDE responses")

    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
