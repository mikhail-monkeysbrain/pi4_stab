#!/usr/bin/env python3
"""Low-rate MSPv2 rangefinder test: 1/2/3 Hz vs 20 Hz hold."""

import argparse
import csv
import os
import select
import struct
import termios
import time
from collections import Counter
from pathlib import Path

from msp_encoder import rangefinder_packet

PORT = "/dev/ttyAMA4"
OUTPUT = Path("/tmp/jtzero_rangefinder_lowrate.csv")

DEBUG_CMD = 254
DEBUG_PERIOD = 0.05
PHASE_DURATION = 6.0
GAP_DURATION = 2.0

PHASES = [
    ("DIRECT_1HZ", 1.0, 1.0),
    ("DIRECT_2HZ", 2.0, 2.0),
    ("DIRECT_3HZ", 3.0, 3.0),
    ("HOLD_1HZ", 1.0, 20.0),
]


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 255
    return crc


def request(cmd):
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
        self.errors = 0
        self.commands = Counter()

    def feed(self, data):
        self.buf.extend(data)
        result = []

        while True:
            pos = self.buf.find(b"$X")
            if pos < 0:
                self.buf.clear()
                break
            if pos:
                del self.buf[:pos]

            if len(self.buf) < 9:
                break

            if self.buf[2] not in (ord(">"), ord("!")):
                del self.buf[0]
                continue

            length = struct.unpack_from("<H", self.buf, 6)[0]
            if length > 256:
                del self.buf[0]
                continue

            total = 9 + length
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

            self.commands[cmd] += 1
            if error:
                self.errors += 1

            result.append((cmd, payload, error))

        return result


def build_schedule():
    schedule = []
    phases = []
    cursor = 1.0

    for name, source_hz, tx_hz in PHASES:
        start = cursor
        end = start + PHASE_DURATION
        phases.append((name, start, end))

        # Measurements change at source_hz.
        # Transmission occurs at tx_hz.
        tx_index = 0

        while True:
            tx_time = start + tx_index / tx_hz
            if tx_time >= end - 1e-8:
                break

            measurement_index = int(
                (tx_time - start) * source_hz + 1e-7
            )
            distance = (
                1.0 if measurement_index % 2 == 0 else 2.0
            )

            schedule.append((
                tx_time,
                name,
                distance,
                measurement_index,
            ))
            tx_index += 1

        cursor = end + GAP_DURATION

    return schedule, phases, cursor


def phase_at(t, phases):
    for name, start, end in phases:
        if start <= t < end:
            return name
    return "GAP"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--transmit",
        action="store_true",
        help="Explicitly enable artificial sensor transmission",
    )
    args = ap.parse_args()

    schedule, phases, duration = build_schedule()

    print("===== RANGEFINDER LOW RATE =====")
    print("PORT:", PORT)
    print("DEBUG RATE: 20 Hz")
    print("TX:", "ENABLED" if args.transmit else "DISABLED")
    print("DURATION:", round(duration, 1), "s")
    print("OUTPUT:", OUTPUT)
    print()

    for name, source_hz, tx_hz in PHASES:
        print(
            name,
            "source=", source_hz,
            "Hz tx=", tx_hz, "Hz",
        )

    fd = os.open(
        PORT,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
    )

    parser = Parser()
    rows = []
    tx_events = []
    sent = Counter()
    debug_requests = 0
    debug_replies = 0
    schedule_index = 0

    try:
        configure(fd)
        start_clock = time.monotonic()
        next_debug = start_clock

        while True:
            now = time.monotonic()
            t = now - start_clock

            if t >= duration:
                break

            if now >= next_debug:
                write_all(fd, request(DEBUG_CMD))
                debug_requests += 1
                next_debug += DEBUG_PERIOD

            if args.transmit:
                while (
                    schedule_index < len(schedule)
                    and t >= schedule[schedule_index][0]
                ):
                    scheduled_t, name, distance, measurement_id = (
                        schedule[schedule_index]
                    )

                    write_all(
                        fd,
                        rangefinder_packet(
                            distance,
                            quality=255,
                        ),
                    )

                    actual_t = time.monotonic() - start_clock

                    tx_events.append({
                        "t_s": round(actual_t, 5),
                        "phase": name,
                        "distance_cm": int(distance * 100),
                        "measurement_id": measurement_id,
                    })

                    sent[name] += 1
                    schedule_index += 1

            ready, _, _ = select.select(
                [fd], [], [], 0.002
            )

            if not ready:
                continue

            data = os.read(fd, 4096)

            for cmd, payload, error in parser.feed(data):
                if cmd != DEBUG_CMD or error:
                    continue
                if len(payload) != 16:
                    continue

                values = struct.unpack("<8h", payload)
                sample_t = time.monotonic() - start_clock

                rows.append({
                    "t_s": round(sample_t, 5),
                    "phase": phase_at(sample_t, phases),
                    "raw_cm": values[1],
                    "corrected_cm": values[2],
                    "debug0": values[0],
                    "debug3": values[3],
                    "debug4": values[4],
                    "debug5": values[5],
                    "debug6": values[6],
                    "debug7": values[7],
                })

                debug_replies += 1

    finally:
        os.close(fd)

    fieldnames = [
        "t_s", "phase", "raw_cm", "corrected_cm",
        "debug0", "debug3", "debug4", "debug5",
        "debug6", "debug7",
    ]

    with OUTPUT.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    tx_output = Path("/tmp/jtzero_rangefinder_lowrate_tx.csv")
    with tx_output.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "t_s", "phase", "distance_cm", "measurement_id"
            ],
        )
        writer.writeheader()
        writer.writerows(tx_events)

    print()
    print("===== TRANSPORT =====")
    print("Debug requests:", debug_requests)
    print("Debug replies:", debug_replies)
    print("CRC errors:", parser.crc_errors)
    print("MSP errors:", parser.errors)
    print("MSP commands:", dict(parser.commands))

    print()
    print("===== PHASE RESULTS =====")

    for name, _, _ in PHASES:
        subset = [r for r in rows if r["phase"] == name]
        events = [e for e in tx_events if e["phase"] == name]

        print()
        print("PHASE:", name)
        print("TX packets:", sent[name])
        print("Debug samples:", len(subset))

        if not subset:
            continue

        values = [r["raw_cm"] for r in subset]
        transitions = []

        previous = subset[0]["raw_cm"]
        for r in subset[1:]:
            if r["raw_cm"] != previous:
                transitions.append(
                    (r["t_s"], previous, r["raw_cm"])
                )
                previous = r["raw_cm"]

        print("Raw min/max:", min(values), max(values))
        print("Raw transitions:", len(transitions))

        for t, old, new in transitions[:20]:
            print(
                "  t=%.4f %d -> %d cm" % (t, old, new)
            )

        # Match each observed value change to the latest
        # preceding transmission of the same value.
        delays = []
        for t, old, new in transitions:
            candidates = [
                e for e in events
                if e["t_s"] <= t
                and e["distance_cm"] == new
                and e["distance_cm"] != old
            ]
            if candidates:
                delays.append(t - candidates[-1]["t_s"])

        if delays:
            print(
                "Observed update delay min/max:",
                round(min(delays), 4),
                round(max(delays), 4),
                "s",
            )

        print("Last debug samples:")
        for r in subset[-4:]:
            print(
                "  t=", r["t_s"],
                "raw=", r["raw_cm"],
                "corrected=", r["corrected_cm"],
                "debug3=", r["debug3"],
            )

    print()
    print("CSV:", OUTPUT)
    print("TX CSV:", tx_output)
    print("UART: CLOSED")
    print(
        "NOTE: Raw debug updates alone do not prove "
        "rangefinder validity."
    )


if __name__ == "__main__":
    main()
