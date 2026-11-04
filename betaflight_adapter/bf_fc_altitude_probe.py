#!/usr/bin/env python3
import csv
import os
import select
import signal
import struct
import sys
import termios
import time
from pathlib import Path

DEVICE = "/dev/ttyAMA4"
BAUD = termios.B230400
OUT = Path("/tmp/bf_fc_altitude.csv")
running = True

def stop(signum, frame):
    global running
    running = False

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

def request(cmd):
    payload = bytes((0x24, 0x4d, 0x3c, 0, cmd, cmd))
    return payload

def configure(fd):
    cfg = termios.tcgetattr(fd)
    cfg[0] = 0
    cfg[1] = 0
    cfg[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    cfg[3] = 0
    cfg[4] = BAUD
    cfg[5] = BAUD
    cfg[6][termios.VMIN] = 0
    cfg[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, cfg)

def parse(buffer):
    results = []
    while True:
        pos = buffer.find(b"$M")
        if pos < 0:
            buffer.clear()
            break
        if pos:
            del buffer[:pos]
        if len(buffer) < 6:
            break
        direction = buffer[2]
        size = buffer[3]
        if direction not in (ord(">"), ord("!")) or size > 128:
            del buffer[0]
            continue
        end = 6 + size
        if len(buffer) < end:
            break
        frame = bytes(buffer[:end])
        del buffer[:end]
        checksum = 0
        for b in frame[3:-1]:
            checksum ^= b
        if checksum != frame[-1]:
            continue
        results.append((direction, frame[4], frame[5:-1]))
    return results

def main():
    if not Path(DEVICE).exists():
        raise RuntimeError("UART не найден: " + DEVICE)

    fd = os.open(DEVICE, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure(fd)
        buffer = bytearray()
        counts = {58: 0, 109: 0}
        last = {58: None, 109: None}
        pending = {}
        start = time.monotonic()
        next_request = start

        with OUT.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "time_s", "command", "status",
                "value", "payload_hex"
            ])

            print("FC_PROBE_START", flush=True)
            print("CSV=" + str(OUT), flush=True)

            while running:
                now = time.monotonic()

                if now >= next_request:
                    cmd = 58 if int((now-start)*5) % 2 == 0 else 109
                    os.write(fd, request(cmd))
                    pending[cmd] = now
                    next_request = now + 0.1

                ready, _, _ = select.select([fd], [], [], 0.02)
                if ready:
                    chunk = os.read(fd, 4096)
                    buffer.extend(chunk)
                    for direction, cmd, data in parse(buffer):
                        if cmd not in counts:
                            continue
                        counts[cmd] += 1
                        status = "OK" if direction == ord(">") else "ERROR"
                        value = None
                        if status == "OK":
                            if cmd == 58 and len(data) >= 4:
                                value = struct.unpack_from("<i", data)[0]
                            elif cmd == 109 and len(data) >= 4:
                                value = struct.unpack_from("<i", data)[0]
                        last[cmd] = value
                        writer.writerow([
                            round(time.monotonic()-start, 3),
                            cmd, status, value, data.hex()
                        ])
                        f.flush()
                        print(
                            f"FC cmd={cmd} status={status} "
                            f"value={value} raw={data.hex()}",
                            flush=True
                        )

        print("FC_PROBE_RESULT", flush=True)
        print("SONAR_RESPONSES=" + str(counts[58]), flush=True)
        print("ALTITUDE_RESPONSES=" + str(counts[109]), flush=True)
        print("LAST_SONAR_RAW=" + str(last[58]), flush=True)
        print("LAST_ALTITUDE_RAW=" + str(last[109]), flush=True)
        print("CSV=" + str(OUT), flush=True)
    finally:
        os.close(fd)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("FC_PROBE_ERROR:", e, file=sys.stderr)
        sys.exit(1)
