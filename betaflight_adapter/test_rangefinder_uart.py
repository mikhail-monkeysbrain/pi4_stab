#!/usr/bin/env python3
"""Limited real-UART MSPv2 rangefinder injection."""

import argparse
import os
import select
import termios
import time

from msp_encoder import rangefinder_packet

PORT = "/dev/ttyAMA4"
BAUD = 115200
COUNT = 40
PERIOD_S = 0.05
DISTANCE_M = 1.0
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--transmit",
        action="store_true",
        help="Explicitly permit real UART transmission",
    )
    args = parser.parse_args()

    packet = rangefinder_packet(
        DISTANCE_M,
        quality=255,
    )

    print("PORT:", PORT)
    print("BAUD:", BAUD)
    print("PACKET:", packet.hex(" "))
    print("COUNT:", COUNT)
    print("RATE: 20 Hz")
    print("DISTANCE: 1500 mm")

    if not args.transmit:
        print("DRY RUN: no UART opened")
        return

    fd = os.open(
        PORT,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
    )

    try:
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

        start = time.monotonic()
        sent = 0

        for i in range(COUNT):
            deadline = start + i * PERIOD_S

            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(remaining, 0.005))

            offset = 0

            while offset < len(packet):
                _, writable, _ = select.select(
                    [], [fd], [], 0.2
                )

                if not writable:
                    raise TimeoutError(
                        "UART write timeout"
                    )

                offset += os.write(
                    fd,
                    packet[offset:],
                )

            sent += 1

        termios.tcdrain(fd)

        print("SENT:", sent)
        print("RESULT: UART WRITE COMPLETED")
        print(
            "NOTE: FC sensor acceptance "
            "not yet verified"
        )

    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
