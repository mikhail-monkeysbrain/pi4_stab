import os
import time
import termios
import struct
import subprocess
import sys
import select

PORT = "/dev/ttyAMA4"

if subprocess.run(
    ["fuser", PORT], capture_output=True
).returncode == 0:
    sys.exit("ERROR: UART занят")

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 255 if crc & 128 else (crc << 1) & 255
    return crc

def range_packet(mm):
    payload = struct.pack("<Bi", 255, mm)
    body = struct.pack("<BHH", 0, 0x1F01, len(payload)) + payload
    return b"$X<" + body + bytes([crc8(body)])

def sonar_request():
    return b"$M<" + bytes([0, 58, 58])

fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

try:
    cfg = termios.tcgetattr(fd)
    cfg[0] = cfg[1] = cfg[3] = 0
    cfg[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    cfg[4] = cfg[5] = termios.B230400
    cfg[6][termios.VMIN] = cfg[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, cfg)

    packet = range_packet(150)
    buffer = bytearray()
    values = []
    sent = 0

    print("TEST_START: RANGE=15cm RATE=10Hz DURATION=10s", flush=True)

    for i in range(100):
        os.write(fd, packet)
        os.write(fd, sonar_request())
        sent += 1

        deadline = time.monotonic() + 0.1
        while time.monotonic() < deadline:
            ready, _, _ = select.select([fd], [], [], 0.01)
            if not ready:
                continue

            buffer.extend(os.read(fd, 4096))

            while True:
                pos = buffer.find(b"$M>")
                if pos < 0:
                    if len(buffer) > 2:
                        del buffer[:-2]
                    break
                if pos:
                    del buffer[:pos]
                if len(buffer) < 6:
                    break

                size = buffer[3]
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

                if frame[4] == 58 and size == 4:
                    value = struct.unpack("<i", frame[5:9])[0]
                    values.append(value)
                    if i % 10 == 0:
                        print("FC_SONAR_CM=", value, flush=True)

    termios.tcdrain(fd)

    print("===== RESULT =====")
    print("PACKETS_SENT=", sent)
    print("SONAR_RESPONSES=", len(values))
    print("SONAR_MIN_CM=", min(values) if values else "NO_DATA")
    print("SONAR_MAX_CM=", max(values) if values else "NO_DATA")
    print("SONAR_LAST_CM=", values[-1] if values else "NO_DATA")

finally:
    os.close(fd)
