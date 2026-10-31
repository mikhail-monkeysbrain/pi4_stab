#!/usr/bin/env python3
"""Read Betaflight MSP_DEBUG (254) over MSPv2."""

import os
import select
import struct
import termios
import time

PORT = "/dev/ttyAMA4"
BAUD = 115200
COMMAND = 254
SAMPLES = 40
INTERVAL_S = 0.1
TIMEOUT_S = 0.3


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


def msp_request(command):
    body = struct.pack("<BHH", 0, command, 0)
    return b"$X<" + body + bytes([crc8(body)])


def configure_uart(fd):
    attrs = termios.tcgetattr(fd)

    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = (
        termios.CS8
        | termios.CLOCAL
        | termios.CREAD
    )
    attrs[3] = 0

    attrs[4] = termios.B115200
    attrs[5] = termios.B115200

    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(
        fd,
        termios.TCSANOW,
        attrs,
    )

    termios.tcflush(
        fd,
        termios.TCIFLUSH,
    )


def read_response(fd, timeout_s):
    deadline = time.monotonic() + timeout_s
    buffer = bytearray()

    while time.monotonic() < deadline:
        ready, _, _ = select.select(
            [fd], [], [], 0.02
        )

        if ready:
            data = os.read(fd, 4096)
            buffer.extend(data)

        while True:
            pos = buffer.find(b"$X")

            if pos < 0:
                buffer.clear()
                break

            if pos > 0:
                del buffer[:pos]

            if len(buffer) < 9:
                break

            direction = buffer[2]

            if direction not in (
                ord(">"),
                ord("!"),
            ):
                del buffer[0]
                continue

            payload_length = struct.unpack_from(
                "<H", buffer, 6
            )[0]

            if payload_length > 256:
                del buffer[0]
                continue

            frame_length = 9 + payload_length

            if len(buffer) < frame_length:
                break

            frame = bytes(
                buffer[:frame_length]
            )

            del buffer[:frame_length]

            if crc8(frame[3:-1]) != frame[-1]:
                return {
                    "status": "CRC_ERROR",
                    "payload": b"",
                }

            command = struct.unpack_from(
                "<H", frame, 4
            )[0]

            if command != COMMAND:
                continue

            if direction == ord("!"):
                return {
                    "status": "MSP_ERROR",
                    "payload": frame[8:-1],
                }

            return {
                "status": "OK",
                "payload": frame[8:-1],
            }

    return {
        "status": "TIMEOUT",
        "payload": b"",
    }


def main():
    fd = os.open(
        PORT,
        os.O_RDWR
        | os.O_NOCTTY
        | os.O_NONBLOCK,
    )

    successful = 0
    failures = 0

    try:
        configure_uart(fd)

        print("===== BETAFIGHT MSP DEBUG =====")
        print("PORT:", PORT)
        print("BAUD:", BAUD)
        print("COMMAND: MSP_DEBUG (254)")
        print("SAMPLES:", SAMPLES)
        print("MODE: READ ONLY")
        print()

        for i in range(SAMPLES):
            start = time.monotonic()

            request = msp_request(COMMAND)

            offset = 0
            while offset < len(request):
                _, writable, _ = select.select(
                    [], [fd], [], TIMEOUT_S
                )

                if not writable:
                    raise TimeoutError(
                        "UART write timeout"
                    )

                offset += os.write(
                    fd,
                    request[offset:],
                )

            result = read_response(
                fd,
                TIMEOUT_S,
            )

            if result["status"] != "OK":
                failures += 1
                print(
                    f"{i:02d}",
                    result["status"],
                )
            else:
                payload = result["payload"]

                if len(payload) != 16:
                    failures += 1
                    print(
                        f"{i:02d}",
                        "UNEXPECTED_LENGTH",
                        len(payload),
                        payload.hex(" "),
                    )
                else:
                    successful += 1

                    values = struct.unpack(
                        "<8h",
                        payload,
                    )

                    print(
                        f"{i:02d}",
                        "debug=", values,
                        "raw_cm=", values[1],
                        "corrected_cm=", values[2],
                        "signal=", values[3],
                    )

            elapsed = time.monotonic() - start
            remaining = INTERVAL_S - elapsed

            if remaining > 0:
                time.sleep(remaining)

        print()
        print("===== SUMMARY =====")
        print("Successful:", successful)
        print("Failures:", failures)
        print("Total:", SAMPLES)

    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
