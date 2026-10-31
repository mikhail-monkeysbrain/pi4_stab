#!/usr/bin/env python3
"""MSPv2 smooth rangefinder: low-rate vs repeated samples."""

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
OUTPUT = Path("/tmp/jtzero_rangefinder_smooth.csv")

DEBUG_CMD = 254
DEBUG_PERIOD = 0.05

PHASES = [
    ("BASELINE", 2.0, 0.0, 0.0),
    ("DIRECT_1HZ", 8.0, 1.0, 1.0),
    ("GAP_1", 2.0, 0.0, 0.0),
    ("DIRECT_2HZ", 8.0, 2.0, 2.0),
    ("GAP_2", 2.0, 0.0, 0.0),
    ("DIRECT_3HZ", 8.0, 3.0, 3.0),
    ("GAP_3", 2.0, 0.0, 0.0),
    ("HOLD_1HZ", 8.0, 1.0, 20.0),
    ("FINAL_GAP", 2.0, 0.0, 0.0),
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
        self.msp_errors = 0
        self.commands = Counter()

    def feed(self, data):
        self.buf.extend(data)
        result = []

        while True:
            pos = self.buf.find(b"$X")
            if pos < 0:
                self.buf[:] = b"$" if self.buf[-1:] == b"$" else b""
                break

            if pos:
                del self.buf[:pos]

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

            self.commands[cmd] += 1

            if error:
                self.msp_errors += 1

            result.append((cmd, payload, error))

        return result


def build_schedule():
    phases = []
    transmissions = []
    cursor = 0.0

    for name, duration, source_hz, tx_hz in PHASES:
        start = cursor
        end = start + duration
        phases.append((name, start, end))

        if tx_hz > 0:
            i = 0
            while start + i / tx_hz < end - 1e-8:
                scheduled = start + i / tx_hz
                age = scheduled - start

                # Real measurements are sampled at source_hz.
                measurement_id = int(age * source_hz + 1e-7)
                measurement_time = measurement_id / source_hz

                # Smooth climb at 5 cm/s, starting at 100 cm.
                distance_m = 1.0 + 0.05 * measurement_time

                transmissions.append((
                    scheduled,
                    name,
                    distance_m,
                    measurement_id,
                ))
                i += 1

        cursor = end

    return phases, transmissions, cursor


def phase_at(t, phases):
    for name, start, end in phases:
        if start <= t < end:
            return name
    return "END"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--transmit", action="store_true")
    args = ap.parse_args()

    phases, transmissions, duration = build_schedule()

    print("===== RANGEFINDER SMOOTH =====")
    print("PORT:", PORT)
    print("DEBUG MODE REQUIRED: RANGEFINDER_QUALITY")
    print("TRANSMIT:", args.transmit)
    print("DURATION:", duration, "s")
    print("OUTPUT:", OUTPUT)

    fd = os.open(
        PORT,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
    )

    parser = Parser()
    rows = []
    tx_events = []
    sent = Counter()
    queries = 0
    replies = 0
    tx_index = 0

    try:
        configure(fd)

        start = time.monotonic()
        next_query = start

        while True:
            now = time.monotonic()
            t = now - start

            if t >= duration:
                break

            if now >= next_query:
                write_all(fd, request(DEBUG_CMD))
                queries += 1
                next_query += DEBUG_PERIOD

            if args.transmit:
                while (
                    tx_index < len(transmissions)
                    and t >= transmissions[tx_index][0]
                ):
                    scheduled, name, distance, measurement_id = (
                        transmissions[tx_index]
                    )

                    write_all(
                        fd,
                        rangefinder_packet(distance, quality=255),
                    )

                    tx_events.append({
                        "t_s": round(time.monotonic() - start, 5),
                        "phase": name,
                        "distance_cm": round(distance * 100, 3),
                        "measurement_id": measurement_id,
                    })

                    sent[name] += 1
                    tx_index += 1

            ready, _, _ = select.select([fd], [], [], 0.002)

            if not ready:
                continue

            for cmd, payload, error in parser.feed(os.read(fd, 4096)):
                if cmd != DEBUG_CMD or error or len(payload) != 16:
                    continue

                values = struct.unpack("<8h", payload)
                sample_t = time.monotonic() - start

                rows.append({
                    "t_s": round(sample_t, 5),
                    "phase": phase_at(sample_t, phases),
                    "raw_cm": values[0],
                    "snr_threshold_reached": values[1],
                    "dynamic_threshold_cm": values[2],
                    "surface_valid": values[3],
                })

                replies += 1

    finally:
        os.close(fd)

    with OUTPUT.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "t_s", "phase", "raw_cm",
                "snr_threshold_reached",
                "dynamic_threshold_cm", "surface_valid",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    tx_output = Path("/tmp/jtzero_rangefinder_smooth_tx.csv")
    with tx_output.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "t_s", "phase", "distance_cm", "measurement_id",
            ],
        )
        writer.writeheader()
        writer.writerows(tx_events)

    print()
    print("===== TRANSPORT =====")
    print("Requests:", queries)
    print("Replies:", replies)
    print("CRC errors:", parser.crc_errors)
    print("MSP errors:", parser.msp_errors)
    print("Commands:", dict(parser.commands))

    print()
    print("===== PHASE SUMMARY =====")

    for name, _, _, _ in PHASES:
        subset = [r for r in rows if r["phase"] == name]

        print()
        print(name)
        print("Sent:", sent[name])
        print("Samples:", len(subset))

        if not subset:
            continue

        raw = [r["raw_cm"] for r in subset]
        valid = [r["surface_valid"] for r in subset]

        print("Raw min/max:", min(raw), max(raw))
        print("Raw unique:", len(set(raw)))
        print("Surface valid 1:", valid.count(1))
        print("Surface valid 0:", valid.count(0))
        print(
            "SNR threshold values:",
            sorted(set(r["snr_threshold_reached"] for r in subset)),
        )
        print(
            "Dynamic threshold values:",
            sorted(set(r["dynamic_threshold_cm"] for r in subset)),
        )

        transitions = []
        previous = subset[0]["raw_cm"]

        for r in subset[1:]:
            if r["raw_cm"] != previous:
                transitions.append(
                    (r["t_s"], previous, r["raw_cm"])
                )
                previous = r["raw_cm"]

        print("Raw transitions:", len(transitions))
        for t, old, new in transitions[:20]:
            print(
                "  t=%.4f %d -> %d cm" % (t, old, new)
            )

        last = subset[-1]
        print(
            "Last:",
            "raw=", last["raw_cm"],
            "valid=", last["surface_valid"],
        )

    print()
    print("CSV:", OUTPUT)
    print("TX CSV:", tx_output)
    print("UART: CLOSED")
    print(
        "NOTE: surface_valid does not directly prove "
        "rangefinderIsHealthy()."
    )


if __name__ == "__main__":
    main()
