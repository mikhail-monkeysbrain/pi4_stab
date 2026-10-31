#!/usr/bin/env python3
"""Betaflight MSPv2 rangefinder dynamic response test."""

import csv
import os
import select
import struct
import termios
import time
from pathlib import Path

from msp_encoder import rangefinder_packet

PORT = "/dev/ttyAMA4"
OUTPUT = Path("/tmp/jtzero_rangefinder_dynamic.csv")

MSP_DEBUG = 254
QUALITY = 255

QUERY_PERIOD = 0.05
TX_PERIOD = 0.05
DURATION = 7.0


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 255
    return crc


def msp_request(cmd):
    body = struct.pack("<BHH", 0, cmd, 0)
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
    termios.tcflush(fd, termios.TCIFLUSH)


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


class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.msp_errors = 0
        self.other = {}

    def feed(self, data):
        self.buf.extend(data)
        result = []

        while True:
            p = self.buf.find(b"$X")

            if p < 0:
                self.buf.clear()
                break

            if p:
                del self.buf[:p]

            if len(self.buf) < 9:
                break

            if self.buf[2] not in (ord(">"), ord("!")):
                del self.buf[0]
                continue

            size = struct.unpack_from("<H", self.buf, 6)[0]

            if size > 256:
                del self.buf[0]
                continue

            total = 9 + size

            if len(self.buf) < total:
                break

            frame = bytes(self.buf[:total])
            del self.buf[:total]

            if crc8(frame[3:-1]) != frame[-1]:
                self.crc_errors += 1
                continue

            cmd = struct.unpack_from("<H", frame, 4)[0]
            payload = frame[8:-1]
            error = frame[2] == ord("!")

            if error:
                self.msp_errors += 1

            if cmd != MSP_DEBUG:
                self.other[cmd] = self.other.get(cmd, 0) + 1

            result.append((cmd, payload, error))

        return result


def phase(t):
    if t < 1:
        return "BASELINE", None
    if t < 3:
        return "RANGE_100", 1.0
    if t < 5:
        return "RANGE_200", 2.0
    return "STOPPED", None


def main():
    fd = os.open(
        PORT,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
    )

    parser = Parser()
    rows = []
    sent_100 = 0
    sent_200 = 0
    queries = 0
    replies = 0
    first_100 = None
    first_200 = None

    try:
        configure(fd)

        start = time.monotonic()
        next_query = start
        next_tx = start + 1.0

        while True:
            now = time.monotonic()
            t = now - start

            if t >= DURATION:
                break

            name, distance = phase(t)

            if now >= next_query:
                write_all(fd, msp_request(MSP_DEBUG))
                queries += 1
                next_query += QUERY_PERIOD

            if distance is not None and now >= next_tx:
                write_all(
                    fd,
                    rangefinder_packet(distance, quality=QUALITY),
                )

                if distance == 1.0:
                    sent_100 += 1
                else:
                    sent_200 += 1

                next_tx += TX_PERIOD

            ready, _, _ = select.select([fd], [], [], 0.002)

            if not ready:
                continue

            data = os.read(fd, 4096)

            for cmd, payload, error in parser.feed(data):
                if cmd != MSP_DEBUG or error or len(payload) != 16:
                    continue

                values = struct.unpack("<8h", payload)
                sample_t = time.monotonic() - start
                sample_phase, _ = phase(sample_t)

                raw = values[1]
                corrected = values[2]

                rows.append({
                    "t_s": round(sample_t, 4),
                    "phase": sample_phase,
                    "raw_cm": raw,
                    "corrected_cm": corrected,
                    "signal": values[3],
                    "cos_tilt": values[4],
                    "min_cos": values[5],
                })

                replies += 1

                if raw == 100 and first_100 is None:
                    first_100 = sample_t

                if raw == 200 and first_200 is None:
                    first_200 = sample_t

        with OUTPUT.open("w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "t_s",
                    "phase",
                    "raw_cm",
                    "corrected_cm",
                    "signal",
                    "cos_tilt",
                    "min_cos",
                ],
            )
            writer.writeheader()
            writer.writerows(rows)

        print("===== RANGEFINDER DYNAMIC =====")
        print("PORT:", PORT)
        print("MSP_DEBUG replies:", replies)
        print("MSP_DEBUG requests:", queries)
        print("CRC errors:", parser.crc_errors)
        print("MSP errors:", parser.msp_errors)
        print("Other MSP commands:", parser.other)
        print("Sent 1000mm:", sent_100)
        print("Sent 2000mm:", sent_200)

        print()
        print("===== RESPONSE =====")
        print("First raw=100 cm:", first_100)
        print("First raw=200 cm:", first_200)

        if first_100 is not None:
            print(
                "100cm response delay (s):",
                round(first_100 - 1.0, 4),
            )

        if first_200 is not None:
            print(
                "200cm response delay (s):",
                round(first_200 - 3.0, 4),
            )

        print()
        print("===== PHASE SUMMARY =====")

        for name in ("BASELINE", "RANGE_100", "RANGE_200", "STOPPED"):
            subset = [r for r in rows if r["phase"] == name]
            raw = [r["raw_cm"] for r in subset]

            print(
                name,
                "samples=", len(subset),
                "raw_min=", min(raw) if raw else None,
                "raw_max=", max(raw) if raw else None,
            )

            for r in subset[:5]:
                print(
                    "  t=", r["t_s"],
                    "raw=", r["raw_cm"],
                    "corrected=", r["corrected_cm"],
                    "signal=", r["signal"],
                )

        print()
        print("CSV:", OUTPUT)
        print("UART: CLOSED")

    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
