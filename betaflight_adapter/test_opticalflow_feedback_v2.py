#!/usr/bin/env python3
"""Bench-only MSPv2 optical flow timing/feedback diagnostic. NOT flight-ready."""
import csv
import struct
import sys
import time
from collections import Counter
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from msp_encoder import crc8_dvb_s2, msp_v2_frame, opticalflow_packet, rangefinder_packet

PORT = '/dev/ttyAMA4'
BAUD = 115200
DEBUG_CMD = 254
PERIOD_NS = 20_000_000
DEBUG_PERIOD_NS = 50_000_000
ACTIVE_START_NS = 2_000_000_000
ACTIVE_END_NS = 6_000_000_000
END_NS = 8_000_000_000
OUT = Path('/tmp/jtzero_opticalflow_feedback_v2.csv')


def parse_frames(buffer):
    frames = []
    while True:
        pos = buffer.find(b'$X')
        if pos < 0:
            if buffer[-1:] == b'$':
                buffer[:] = b'$'
            else:
                buffer.clear()
            break
        if pos:
            del buffer[:pos]
        if len(buffer) < 9:
            break
        if buffer[2] not in (ord('>'), ord('!')):
            del buffer[0]
            continue
        size = struct.unpack_from('<H', buffer, 6)[0]
        if size > 256:
            del buffer[0]
            continue
        length = 9 + size
        if len(buffer) < length:
            break
        frame = bytes(buffer[:length])
        if crc8_dvb_s2(frame[3:-1]) != frame[-1]:
            del buffer[0]
            continue
        command = struct.unpack_from('<H', frame, 4)[0]
        frames.append((command, frame[8:-1], frame[2] == ord('!')))
        del buffer[:length]
    return frames


def phase(t):
    if t < 2.0:
        return 'BASELINE'
    if t < 6.0:
        return 'ACTIVE'
    return 'STOPPED'


def main():
    print('BENCH TEST ONLY: synthetic rangefinder + optical flow. PROPS REMOVED.')
    print('UART:', PORT, BAUD, '| duration 8 s | active 2-6 s')
    print('Flow: motion_x=2, motion_y=0, quality=255; range=1000mm; nominal 50 Hz')
    print('CSV:', OUT)
    events = []
    stats = Counter()
    tx_times = []
    rxbuf = bytearray()

    with serial.Serial(PORT, BAUD, timeout=0, write_timeout=0.2) as ser:
        ser.reset_input_buffer()
        start = time.monotonic_ns()
        next_tx = start + ACTIVE_START_NS
        next_dbg = start

        while True:
            now = time.monotonic_ns()
            t_ns = now - start
            if t_ns >= END_NS:
                break

            if ACTIVE_START_NS <= t_ns < ACTIVE_END_NS and now >= next_tx:
                # Do not burst catch-up packets after scheduler stalls.
                if now - next_tx >= PERIOD_NS:
                    missed = (now - next_tx) // PERIOD_NS
                    stats['missed_tx_slots'] += missed
                    next_tx += missed * PERIOD_NS

                try:
                    ser.write(rangefinder_packet(1.0, 255))
                    stats['range_sent'] += 1
                    ser.write(opticalflow_packet(2, 0, 255))
                    stats['flow_sent'] += 1

                    stamp = time.monotonic_ns()
                    tx_times.append(stamp)
                    events.append([
                        round((stamp-start)/1e9, 6),
                        'TX', 'ACTIVE', 0x1F02,
                        '', '', '', '', '', '', '', '', ''
                    ])
                except serial.SerialTimeoutException:
                    stats['tx_timeouts'] += 1

                next_tx += PERIOD_NS

            if now >= next_dbg:
                try:
                    ser.write(msp_v2_frame(DEBUG_CMD, b''))
                    stats['debug_requests'] += 1
                except serial.SerialTimeoutException:
                    stats['tx_timeouts'] += 1

                # Skip missed query slots rather than catching up in bursts.
                next_dbg += (
                    (now - next_dbg) // DEBUG_PERIOD_NS + 1
                ) * DEBUG_PERIOD_NS

            available = ser.in_waiting
            if available:
                rxbuf.extend(ser.read(available))

                for command, payload, is_error in parse_frames(rxbuf):
                    stamp = time.monotonic_ns()
                    t = (stamp - start) / 1e9

                    if is_error:
                        stats['msp_errors'] += 1
                        continue

                    if command == 0x1F01:
                        stats['range_replies'] += 1
                    elif command == 0x1F02:
                        stats['flow_replies'] += 1
                    elif command == DEBUG_CMD and len(payload) >= 16:
                        stats['debug_replies'] += 1
                        vals = struct.unpack_from('<8h', payload)
                        events.append([
                            round(t, 6), 'DEBUG',
                            phase(t), command, *vals
                        ])

            time.sleep(0.001)

    with OUT.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            'time_s', 'event', 'phase', 'command',
            'd0', 'd1', 'd2', 'd3',
            'd4', 'd5', 'd6', 'd7'
        ])
        for e in events:
            w.writerow(e[:12])

    print('\nTRANSPORT')
    for k in (
        'range_sent', 'flow_sent',
        'range_replies', 'flow_replies',
        'debug_requests', 'debug_replies',
        'msp_errors', 'tx_timeouts',
        'missed_tx_slots'
    ):
        print(f'{k}: {stats[k]}')

    if len(tx_times) > 1:
        dt = [
            (b-a)/1e6
            for a, b in zip(tx_times, tx_times[1:])
        ]
        print(
            f'TX interval ms: min={min(dt):.3f} '
            f'mean={sum(dt)/len(dt):.3f} '
            f'max={max(dt):.3f}'
        )

    print('\nDEBUG BY PHASE (stale values may persist after TX stops)')

    for name in ('BASELINE', 'ACTIVE', 'STOPPED'):
        rows = [
            e for e in events
            if e[1] == 'DEBUG' and e[2] == name
        ]
        print(name, 'samples=', len(rows))

        if rows:
            for idx in (0, 1, 4, 5, 6, 7):
                vals = [int(e[4+idx]) for e in rows]
                print(
                    f' d{idx}: min={min(vals)} '
                    f'max={max(vals)} last={vals[-1]}'
                )

    print('CSV:', OUT, '| UART CLOSED')


if __name__ == '__main__':
    main()
