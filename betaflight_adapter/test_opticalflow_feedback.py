#!/usr/bin/env python3
import sys
import time
import struct
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from msp_encoder import (
    msp_v2_frame,
    rangefinder_packet,
    opticalflow_packet,
    crc8_dvb_s2,
)

PORT = "/dev/ttyAMA4"
BAUD = 115200
DEBUG_CMD = 254

def read_frames(buffer):
    frames = []
    while True:
        start = buffer.find(b"$X")
        if start < 0:
            buffer.clear()
            break
        if start:
            del buffer[:start]
        if len(buffer) < 9:
            break
        if buffer[2] not in (ord(">"), ord("!")):
            del buffer[0]
            continue
        size = struct.unpack_from("<H", buffer, 6)[0]
        if size > 256:
            del buffer[0]
            continue
        total = 9 + size
        if len(buffer) < total:
            break
        frame = bytes(buffer[:total])
        if crc8_dvb_s2(frame[3:-1]) != frame[-1]:
            del buffer[0]
            continue
        command = struct.unpack_from("<H", frame, 4)[0]
        frames.append((command, frame[8:-1], frame[2] == ord("!")))
        del buffer[:total]
    return frames

def main():
    print("===== OPTICALFLOW MSP FEEDBACK =====")
    print("PORT:", PORT)
    print("DEBUG MODE: OPTICALFLOW")
    print("Duration: 8 seconds")
    print("TX: range 1000mm + flow X=2 Y=0 at 50Hz")
    print()

    rx = bytearray()
    stats = {"range_ack": 0, "flow_ack": 0,
             "debug": 0, "errors": 0}
    samples = []

    with serial.Serial(PORT, BAUD, timeout=0, write_timeout=0.2) as ser:
        start = time.monotonic()
        next_tx = start + 2.0
        next_debug = start
        sent_range = 0
        sent_flow = 0

        while True:
            now = time.monotonic()
            elapsed = now - start
            if elapsed >= 8.0:
                break

            if 2.0 <= elapsed < 6.0 and now >= next_tx:
                ser.write(rangefinder_packet(1.0, 255))
                ser.write(opticalflow_packet(2, 0, 255))
                sent_range += 1
                sent_flow += 1
                next_tx += 0.02

            if now >= next_debug:
                ser.write(msp_v2_frame(DEBUG_CMD, b""))
                next_debug += 0.05

            incoming = ser.read(ser.in_waiting or 0)
            if incoming:
                rx.extend(incoming)
                for command, payload, error in read_frames(rx):
                    if error:
                        stats["errors"] += 1
                    elif command == 0x1F01:
                        stats["range_ack"] += 1
                    elif command == 0x1F02:
                        stats["flow_ack"] += 1
                    elif command == DEBUG_CMD and len(payload) >= 16:
                        values = struct.unpack_from("<8h", payload)
                        stats["debug"] += 1
                        samples.append((elapsed, values))

            time.sleep(0.001)

    print("===== TRANSPORT =====")
    print("Range sent:", sent_range)
    print("Flow sent:", sent_flow)
    print("Range ACK:", stats["range_ack"])
    print("Flow ACK:", stats["flow_ack"])
    print("Debug replies:", stats["debug"])
    print("MSP errors:", stats["errors"])

    print("\n===== DEBUG PHASES =====")
    for name, lo, hi in [
        ("BASELINE", 0, 2),
        ("ACTIVE", 2, 6),
        ("STOPPED", 6, 8),
    ]:
        rows = [v for t, v in samples if lo <= t < hi]
        print(name, "samples:", len(rows))
        if rows:
            for i in (0, 1, 4, 5, 6, 7):
                values = [r[i] for r in rows]
                print(f"  debug[{i}]: min={min(values)} max={max(values)}")
            print("  last:", rows[-1])

    print("\nUART: CLOSED")

if __name__ == "__main__":
    main()
