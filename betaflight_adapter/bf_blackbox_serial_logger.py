#!/usr/bin/env python3
import os
import select
import signal
import termios
import time
from datetime import datetime
from pathlib import Path

DEVICE = "/dev/ttyAMA5"
BAUD = termios.B1500000
OUTDIR = Path("/home/vio/monkeysStab_runs/blackbox")

running = True

def stop(signum, frame):
    global running
    running = False

signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)

OUTDIR.mkdir(parents=True, exist_ok=True)

fd = os.open(DEVICE, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
old = termios.tcgetattr(fd)

try:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = BAUD
    attrs[5] = BAUD
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    output = None
    last_data = 0
    size = 0

    print("BLACKBOX LOGGER READY", DEVICE, flush=True)

    while running:
        readable, _, _ = select.select([fd], [], [], 0.5)
        if readable:
            try:
                data = os.read(fd, 65536)
            except BlockingIOError:
                continue

            if data:
                now = time.monotonic()

                if output is None:
                    filename = OUTDIR / (
                        "blackbox_" +
                        datetime.now().strftime("%Y%m%d_%H%M%S") +
                        ".bbl"
                    )
                    output = open(filename, "wb", buffering=0)
                    size = 0
                    print("BLACKBOX START", filename, flush=True)

                output.write(data)
                size += len(data)
                last_data = now

        if output is not None and time.monotonic() - last_data > 5:
            output.close()
            print("BLACKBOX SAVED", size, "bytes", flush=True)
            output = None

    if output is not None:
        output.close()
finally:
    termios.tcsetattr(fd, termios.TCSANOW, old)
    os.close(fd)
