#!/usr/bin/env python3
"""Single-port MSPv2 rangefinder injection and debug readback."""

import argparse
import csv
import os
import select
import struct
import termios
import time
from pathlib import Path

from msp_encoder import rangefinder_packet

PORT = "/dev/ttyAMA4"
MSP_DEBUG = 254

DISTANCE_M = 1.0
QUALITY = 255
TX_COUNT = 40
TX_PERIOD_S = 0.05
QUERY_PERIOD_S = 0.10

OUTPUT = Path("/tmp/jtzero_rangefinder_feedback_debug.csv")


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (
                (crc << 1)
                ^ (0xD5 if crc & 0x80 else 0)
            ) & 0xFF
    return crc


def request(command):
    body = struct.pack("<BHH", 0, command, 0)
    return b"$X<" + body + bytes([crc8(body)])


def configure_uart(fd):
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


class MspParser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.msp_errors = 0

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

            direction = self.buffer[2]
            if direction not in (ord(">"), ord("!")):
                del self.buffer[0]
                continue

            length = struct.unpack_from(
                "<H", self.buffer, 6
            )[0]

            if length > 256:
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
            error = direction == ord("!")

            if error:
                self.msp_errors += 1

            frames.append((command, payload, error))

        return frames


def phase_at(t):
    if t < 1.0:
        return "BEFORE"
    if t < 3.0:
        return "DURING"
    return "AFTER"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--transmit",
        action="store_true",
        help="Permit real rangefinder injection",
    )
    args = ap.parse_args()

    print("===== RANGEFINDER FEEDBACK =====")
    print("PORT:", PORT)
    print("MSP_DEBUG:", MSP_DEBUG)
    print("DISTANCE:", int(DISTANCE_M * 1000), "mm")
    print("TX RATE: 20 Hz")
    print("TX COUNT:", TX_COUNT)
    print("TX:", "ENABLED" if args.transmit else "DISABLED")
    print("OUTPUT:", OUTPUT)
    print()

    fd = os.open(
        PORT,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
    )

    parser = MspParser()
    rows = []

    sent = 0
    queries = 0
    replies = 0
    unexpected = 0

    try:
        configure_uart(fd)

        start = time.monotonic()
        next_query = start
        next_tx = start + 1.0

        while True:
            now = time.monotonic()
            t = now - start

            if t >= 4.0:
                break

            # One request every 100 ms.
            if now >= next_query:
                write_all(fd, request(MSP_DEBUG))
                queries += 1
                next_query += QUERY_PERIOD_S

            # Forty rangefinder packets during seconds 1..3.
            if (
                args.transmit
                and 1.0 <= t < 3.0
                and sent < TX_COUNT
                and now >= next_tx
            ):
                write_all(
                    fd,
                    rangefinder_packet(
                        DISTANCE_M,
                        quality=QUALITY,
                    ),
                )
                sent += 1
                next_tx += TX_PERIOD_S

            ready, _, _ = select.select(
                [fd], [], [], 0.003
            )

            if not ready:
                continue

            data = os.read(fd, 4096)

            for command, payload, error in parser.feed(data):
                if command != MSP_DEBUG:
                    unexpected += 1
                    if unexpected <= 5:
                        print("EXTRA RESPONSE:", f"command=0x{command:04X}", "error=", error, "payload=", payload.hex(" "))
                    continue

                if error or len(payload) != 16:
                    unexpected += 1
                    continue

                values = struct.unpack("<8h", payload)
                timestamp = time.monotonic() - start
                phase = phase_at(timestamp)

                rows.append({
                    "t_s": round(timestamp, 4),
                    "phase": phase,
                    "raw_cm": values[1],
                    "corrected_cm": values[2],
                    "signal": values[3],
                    "debug0": values[0],
                    "debug4": values[4],
                    "debug5": values[5],
                    "debug6": values[6],
                    "debug7": values[7],
                })

                replies += 1

        with OUTPUT.open("w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "t_s",
                    "phase",
                    "raw_cm",
                    "corrected_cm",
                    "signal",
                    "debug0",
                    "debug4",
                    "debug5",
                    "debug6",
                    "debug7",
                ],
            )
            writer.writeheader()
            writer.writerows(rows)

        print("===== RESULTS =====")
        print("Range packets sent:", sent)
        print("Debug requests:", queries)
        print("Debug responses:", replies)
        print("CRC errors:", parser.crc_errors)
        print("MSP errors:", parser.msp_errors)
        print("Unexpected responses:", unexpected)
        print()

        for phase in ("BEFORE", "DURING", "AFTER"):
            subset = [r for r in rows if r["phase"] == phase]

            print("PHASE:", phase)
            print("Samples:", len(subset))

            if subset:
                raw_values = [r["raw_cm"] for r in subset]
                corrected = [
                    r["corrected_cm"] for r in subset
                ]

                print(
                    "raw_cm min/max:",
                    min(raw_values),
                    max(raw_values),
                )
                print(
                    "corrected_cm min/max:",
                    min(corrected),
                    max(corrected),
                )

                for r in subset[:5]:
                    print(
                        " t=", r["t_s"],
                        "raw=", r["raw_cm"],
                        "corrected=", r["corrected_cm"],
                        "signal=", r["signal"],
                        "debug4=", r["debug4"],
                        "debug5=", r["debug5"],
                    )

            print()

        print("CSV:", OUTPUT)

        if replies == 0:
            raise SystemExit("FAIL: no MSP_DEBUG responses")

        if args.transmit and sent != TX_COUNT:
            raise SystemExit(
                f"FAIL: transmitted {sent}/{TX_COUNT}"
            )

        print("UART: CLOSED AFTER TEST")
        print("FC SENSOR ACCEPTANCE: REQUIRES ANALYSIS")

    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
