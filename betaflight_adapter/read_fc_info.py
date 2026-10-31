#!/usr/bin/env python3
"""Read-only Betaflight MSPv2 configuration probe."""

import os
import select
import struct
import termios
import time

PORT = "/dev/ttyAMA4"
BAUD = 115200

COMMANDS = (
    (1, "MSP_API_VERSION"),
    (2, "MSP_FC_VARIANT"),
    (4, "MSP_BOARD_INFO"),
    (5, "MSP_BUILD_INFO"),
    (36, "MSP_FEATURE_CONFIG"),
    (54, "MSP_SENSOR_CONFIG"),
)


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


def make_request(command):
    header = struct.pack("<BHH", 0, command, 0)
    return b"$X<" + header + bytes([crc8(header)])


def configure(fd):
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


def read_frame(fd, timeout=1.0):
    deadline = time.monotonic() + timeout
    buffer = bytearray()

    while time.monotonic() < deadline:
        ready, _, _ = select.select([fd], [], [], 0.05)

        if ready:
            data = os.read(fd, 4096)
            buffer.extend(data)

        # Search for MSPv2 response header.
        while len(buffer) >= 3:
            if buffer[:3] in (b"$X>", b"$X!"):
                break
            del buffer[0]

        if len(buffer) < 9:
            continue

        payload_size = struct.unpack_from("<H", buffer, 6)[0]

        if payload_size > 4096:
            raise ValueError("Invalid MSP payload size")

        frame_size = 9 + payload_size

        if len(buffer) < frame_size:
            continue

        frame = bytes(buffer[:frame_size])

        expected_crc = crc8(frame[3:-1])
        actual_crc = frame[-1]

        if expected_crc != actual_crc:
            raise ValueError(
                f"CRC mismatch: {actual_crc:02x} "
                f"!= {expected_crc:02x}"
            )

        command = struct.unpack_from("<H", frame, 4)[0]
        payload = frame[8:-1]
        is_error = frame[:3] == b"$X!"

        return command, payload, is_error

    return None


def main():
    fd = os.open(
        PORT,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
    )

    try:
        configure(fd)

        print("===== BETAFIGHT MSPv2 READ ONLY =====")
        print("PORT:", PORT)
        print("BAUD:", BAUD)
        print("SENSOR DATA TX: DISABLED")
        print("CONFIG WRITE: DISABLED")
        print()

        for command, name in COMMANDS:
            # Clear unread input before the next request.
            termios.tcflush(fd, termios.TCIFLUSH)

            request = make_request(command)
            os.write(fd, request)

            result = read_frame(fd)

            print(f"===== {name} ({command}) =====")

            if result is None:
                print("NO RESPONSE")
                continue

            response_command, payload, is_error = result

            if response_command != command:
                print(
                    "UNEXPECTED RESPONSE COMMAND:",
                    response_command,
                )
                continue

            if is_error:
                print("MSP ERROR / UNSUPPORTED")
                continue

            print("CRC: OK")
            print("PAYLOAD:", payload.hex(" "))
            print("LENGTH:", len(payload))

            if command == 1 and len(payload) >= 3:
                print(
                    "API:",
                    f"{payload[1]}.{payload[2]}"
                )

            elif command == 2:
                print(
                    "FC VARIANT:",
                    payload.decode("ascii", errors="replace")
                )

            elif command == 36 and len(payload) >= 4:
                mask = struct.unpack_from("<I", payload)[0]
                print("FEATURE MASK:", hex(mask))

            print()

    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
