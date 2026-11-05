#!/usr/bin/env python3
"""IRC Tramp: чтение ответов VTX через UART5."""
import argparse
import time


def packet(command, value=0):
    p = bytearray(16)
    p[0] = 0x0F
    p[1] = ord(command)
    p[2] = value & 0xFF
    p[3] = (value >> 8) & 0xFF
    p[14] = sum(p[1:14]) & 0xFF
    return bytes(p)


def valid(p):
    return (
        len(p) == 16
        and p[0] == 0x0F
        and p[15] == 0
        and p[14] == (sum(p[1:14]) & 0xFF)
    )


def u16(p, offset):
    return p[offset] | (p[offset + 1] << 8)


def explain(p):
    cmd = chr(p[1]) if 32 <= p[1] < 127 else f'0x{p[1]:02x}'

    if cmd == 'r':
        return (
            f'capabilities: min_freq={u16(p, 2)} MHz, '
            f'max_freq={u16(p, 4)} MHz, '
            f'max_power={u16(p, 6)} mW'
        )

    if cmd == 'v':
        return (
            f'status: frequency={u16(p, 2)} MHz, '
            f'configured_power={u16(p, 4)} mW, '
            f'control_mode={p[6]}, pit_mode={p[7]}'
        )

    if cmd == 's':
        return (
            f'temperature/raw status: value={u16(p, 2)} '
            '(device-specific units)'
        )

    return f'unknown response command={cmd}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyAMA5')
    ap.add_argument('--timeout', type=float, default=0.5)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    if not 0 < args.timeout <= 5:
        ap.error('--timeout must be >0 and <=5 seconds')

    if args.dry_run:
        for cmd in 'rvs':
            print(f'QUERY {cmd}: {packet(cmd).hex(" ")}')
        return

    import serial

    with serial.Serial(
        args.port,
        9600,
        timeout=0.02,
        write_timeout=1
    ) as uart:

        for cmd in 'rvs':
            uart.reset_input_buffer()
            uart.write(packet(cmd))
            uart.flush()

            deadline = time.monotonic() + args.timeout
            buf = bytearray()
            responses = []

            while time.monotonic() < deadline:
                buf.extend(uart.read(64))

                while len(buf) >= 16:
                    start = buf.find(b'\x0f')

                    if start < 0:
                        buf.clear()
                        break

                    if start:
                        del buf[:start]

                    if len(buf) < 16:
                        break

                    frame = bytes(buf[:16])
                    del buf[:16]

                    if valid(frame):
                        responses.append(frame)
                    else:
                        print(f'BAD FRAME: {frame.hex(" ")}')

            print(f'QUERY {cmd}: {len(responses)} valid frame(s)')

            for frame in responses:
                print('  RAW:', frame.hex(' '))

                if frame == packet(cmd):
                    print('  TX echo (not a VTX response)')
                else:
                    print(' ', explain(frame))

            if not responses and buf:
                print('  INCOMPLETE:', buf.hex(' '))


if __name__ == '__main__':
    main()
