#!/usr/bin/env python3
"""Read-only MSP API version probe over UART."""

import argparse
import os
import select
import termios
import time


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 0xFF
    return crc


def request_api_version():
    # MSPv2, command 1, empty payload.
    header = bytes([0, 1, 0, 0, 0])
    return b"$X<" + header + bytes([crc8(header)])


def configure_uart(fd):
    attrs = termios.tcgetattr(fd)

    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0

    attrs[4] = termios.B115200
    attrs[5] = termios.B115200

    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        default="/dev/ttyAMA4",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
    )
    args = parser.parse_args()

    fd = os.open(
        args.port,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
    )

    try:
        configure_uart(fd)

        request = request_api_version()

        print("PORT:", args.port)
        print("BAUD: 115200")
        print("REQUEST:", request.hex(" "))

        os.write(fd, request)

        deadline = time.monotonic() + args.timeout
        response = bytearray()

        while time.monotonic() < deadline:
            ready, _, _ = select.select(
                [fd], [], [], 0.1
            )

            if ready:
                response.extend(os.read(fd, 4096))

                if len(response) >= 9:
                    break

        if not response:
            print("RESULT: NO RESPONSE")
            raise SystemExit(2)

        print("RESPONSE:", response.hex(" "))

        if response.startswith(b"$X>"):
            print("RESULT: MSPv2 RESPONSE RECEIVED")
        elif response.startswith(b"$M>"):
            print("RESULT: MSPv1 RESPONSE RECEIVED")
        else:
            print("RESULT: UNKNOWN RESPONSE")

    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
