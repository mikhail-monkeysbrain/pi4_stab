#!/usr/bin/env python3
"""Set ArduPilot VTX_POWER via MAVLink TCP router, verify PARAM_VALUE.

Requires only Python standard library. Does not access UART or IRC Tramp.
"""
import argparse
import socket
import struct
import sys
import time

PARAM_SET = 23
PARAM_VALUE = 22
MAV_PARAM_TYPE_REAL32 = 9
PARAM_SET_CRC_EXTRA = 168
PARAM_VALUE_CRC_EXTRA = 220


def x25(data):
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def make_param_set(value, seq, target_system, target_component):
    payload = struct.pack('<fBB16sB', float(value), target_system,
                          target_component, b'VTX_POWER', MAV_PARAM_TYPE_REAL32)
    header = struct.pack('<BBBBB', len(payload), seq & 255, 245, 190, PARAM_SET)
    checksum = x25(header + payload + bytes([PARAM_SET_CRC_EXTRA]))
    return b'\xFE' + header + payload + struct.pack('<H', checksum)


def extract_mavlink_frames(buffer):
    """Yield MAVLink 1/2 frames; leave incomplete tail in buffer."""
    while buffer:
        first_v1 = buffer.find(b'\xFE')
        first_v2 = buffer.find(b'\xFD')
        starts = [x for x in (first_v1, first_v2) if x >= 0]
        if not starts:
            buffer.clear()
            return
        start = min(starts)
        if start:
            del buffer[:start]
        magic = buffer[0]
        header_size = 6 if magic == 0xFE else 10
        if len(buffer) < header_size:
            return
        payload_size = buffer[1]
        signature_size = 13 if magic == 0xFD and buffer[2] & 1 else 0
        length = header_size + payload_size + 2 + signature_size
        if len(buffer) < length:
            return
        frame = bytes(buffer[:length])
        del buffer[:length]
        yield frame


def decode_param_value(frame):
    magic = frame[0]
    if magic == 0xFE:
        if frame[5] != PARAM_VALUE:
            return None
        sysid, compid = frame[3], frame[4]
        head = 6
        crc_extra = frame[1:6]
    else:
        msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
        if msgid != PARAM_VALUE:
            return None
        sysid, compid = frame[5], frame[6]
        head = 10
        crc_extra = frame[1:10]
    payload = frame[head:head + frame[1]]
    if len(payload) < 25:
        return None
    received_crc = struct.unpack_from('<H', frame, head + frame[1])[0]
    if x25(crc_extra + payload + bytes([PARAM_VALUE_CRC_EXTRA])) != received_crc:
        return None
    value, count, index, name, param_type = struct.unpack_from('<fHH16sB', payload)
    return sysid, compid, name.rstrip(b'\0'), value, param_type


def main():
    ap = argparse.ArgumentParser(description='Set FC VTX_POWER via MAVLink and verify response')
    ap.add_argument('power', type=int, help='Requested power, mW (1..1000)')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5760)
    ap.add_argument('--sysid', type=int, default=1, help='Target FC system ID')
    ap.add_argument('--compid', type=int, default=1, help='Target FC component ID')
    ap.add_argument('--timeout', type=float, default=5.0)
    args = ap.parse_args()
    if not 1 <= args.power <= 1000:
        ap.error('power must be 1..1000 mW')
    if not 1 <= args.sysid <= 255 or not 1 <= args.compid <= 255:
        ap.error('sysid and compid must be 1..255')
    if args.timeout <= 0:
        ap.error('timeout must be positive')

    packet = make_param_set(args.power, 0, args.sysid, args.compid)
    print(f'Sending VTX_POWER={args.power} mW to FC {args.sysid}/{args.compid} via {args.host}:{args.port}', flush=True)
    try:
        with socket.create_connection((args.host, args.port), timeout=3) as sock:
            sock.settimeout(0.5)
            # Drain already queued data briefly to avoid treating stale PARAM_VALUE as confirmation.
            drain_until = time.monotonic() + 0.25
            while time.monotonic() < drain_until:
                try:
                    sock.recv(4096)
                except socket.timeout:
                    break
            sock.sendall(packet)
            deadline = time.monotonic() + args.timeout
            buffer = bytearray()
            while time.monotonic() < deadline:
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    print('ERROR: router closed TCP connection', file=sys.stderr)
                    return 2
                buffer.extend(data)
                for frame in extract_mavlink_frames(buffer):
                    item = decode_param_value(frame)
                    if item is None:
                        continue
                    sysid, compid, name, value, param_type = item
                    if sysid != args.sysid or compid != args.compid or name != b'VTX_POWER':
                        continue
                    print(f'FC PARAM_VALUE: VTX_POWER={value:g}, type={param_type}, source={sysid}/{compid}')
                    if abs(value - args.power) < 0.5:
                        print('OK: FC confirmed requested parameter value')
                        return 0
                    print('ERROR: FC reported a different value', file=sys.stderr)
                    return 3
            print('ERROR: no VTX_POWER PARAM_VALUE confirmation before timeout', file=sys.stderr)
            return 4
    except OSError as exc:
        print(f'ERROR: TCP connection failed: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
