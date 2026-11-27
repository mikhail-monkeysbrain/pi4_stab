#!/usr/bin/env python3
import argparse
import serial
import fcntl
from vtx_fc_safety import require_disarmed
from tramp_driver import LOCKFILE
import time

def packet(command, value):
    p = bytearray(16)
    p[0] = 0x0F
    p[1] = ord(command)
    p[2] = value & 0xFF
    p[3] = (value >> 8) & 0xFF
    p[14] = sum(p[1:14]) & 0xFF
    return bytes(p)

parser = argparse.ArgumentParser()
parser.add_argument("--freq", type=int)
parser.add_argument("--power", type=int)
parser.add_argument("--port", default="/dev/ttyS2")
args = parser.parse_args()

if args.freq is None and args.power is None:
    parser.error("Specify --freq or --power")

if args.freq is not None and not 5000 <= args.freq <= 6000:
    parser.error("Frequency must be 5000..6000 MHz")

if args.power is not None and not 0 <= args.power <= 20000:
    parser.error("Power must be 0..20000 mW")

with open(LOCKFILE, "w") as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    require_disarmed()

    with serial.Serial(args.port, 9600, timeout=0.2) as uart:
        commands = []
        if args.freq is not None:
            commands.append(("F", args.freq))
        if args.power is not None:
            commands.append(("P", args.power))

        for cmd, value in commands:
            for _ in range(3):
                uart.write(packet(cmd, value))
                uart.flush()
                time.sleep(0.15)
            print(f"SENT {cmd}={value} (not confirmed)")
