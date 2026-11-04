import os
import time
import termios
import struct
import subprocess
import sys
import select

FC_PORT = "/dev/ttyAMA4"
LUNA_PORT = "/dev/serial0"

RATE_HZ = 10
MAX_AGE_S = 0.75

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def mspv2_rangefinder(mm):
    payload = struct.pack("<Bi", 255, mm)
    header = struct.pack("<BHH", 0, 0x1F01, len(payload))
    body = header + payload
    return b"$X<" + body + bytes([crc8(body)])

def open_uart(port, baud):
    fd = os.open(
        port,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
    )
    cfg = termios.tcgetattr(fd)
    cfg[0] = 0
    cfg[1] = 0
    cfg[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    cfg[3] = 0
    cfg[4] = baud
    cfg[5] = baud
    cfg[6][termios.VMIN] = 0
    cfg[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, cfg)
    return fd

for port in (FC_PORT, LUNA_PORT):
    owners = subprocess.run(
        ["fuser", port],
        capture_output=True,
        text=True
    )
    if owners.returncode == 0:
        sys.exit(
            "ERROR: UART занят: " + port +
            " " + owners.stdout.strip()
        )

fc = None
luna = None

try:
    fc = open_uart(FC_PORT, termios.B230400)
    luna = open_uart(LUNA_PORT, termios.B115200)

    buffer = bytearray()
    last_mm = None
    last_time = 0.0

    luna_count = 0
    tx_count = 0

    next_tx = time.monotonic()

    print("TF_LUNA_TO_BETAFLIGHT_START", flush=True)
    print("FC=/dev/ttyAMA4 230400")
    print("LUNA=/dev/serial0 115200")
    print("RANGE_TX=10Hz")
    print("FLOW_TX=DISABLED")
    print("STOP=CTRL+C", flush=True)

    while True:
        now = time.monotonic()

        ready, _, _ = select.select([luna], [], [], 0.01)

        if ready:
            data = os.read(luna, 4096)
            buffer.extend(data)

            while len(buffer) >= 9:
                pos = buffer.find(b"\x59\x59")

                if pos < 0:
                    buffer.clear()
                    break

                if pos:
                    del buffer[:pos]

                if len(buffer) < 9:
                    break

                frame = bytes(buffer[:9])

                if (sum(frame[:8]) & 0xFF) != frame[8]:
                    del buffer[0]
                    continue

                distance_cm = frame[2] | (frame[3] << 8)
                strength = frame[4] | (frame[5] << 8)

                del buffer[:9]

                if not (5 <= distance_cm <= 800):
                    continue

                if strength == 0:
                    continue

                last_mm = distance_cm * 10
                last_time = time.monotonic()
                luna_count += 1

                print(
                    f"LUNA={last_mm / 10:.0f}cm "
                    f"STRENGTH={strength}",
                    flush=True
                )

        now = time.monotonic()

        if now >= next_tx:
            next_tx = now + 1.0 / RATE_HZ

            if (
                last_mm is not None
                and now - last_time <= MAX_AGE_S
            ):
                packet = mspv2_rangefinder(last_mm)

                written = os.write(fc, packet)

                if written != len(packet):
                    raise RuntimeError(
                        f"Short UART write: {written}/{len(packet)}"
                    )

                tx_count += 1

                if tx_count % 10 == 0:
                    print(
                        f"TX={last_mm / 10:.0f}cm "
                        f"PACKETS={tx_count}",
                        flush=True
                    )

except KeyboardInterrupt:
    print("\nSTOPPED_BY_USER")

finally:
    if fc is not None:
        termios.tcdrain(fc)
        os.close(fc)
    if luna is not None:
        os.close(luna)

    print("===== RESULT =====")
    print("LUNA_RX=", luna_count)
    print("RANGE_TX=", tx_count)
    print("FLOW_TX=0")
