#!/usr/bin/env python3
"""JT-Zero MSPv2 UART sender. No automatic hardware access."""

import math
import time

from msp_encoder import AngularMotionQuantizer, opticalflow_packet


class MspUartSender:
    def __init__(
        self,
        device="/dev/ttyAMA4",
        baudrate=115200,
        period_ns=20_000_000,
        max_age_ns=30_000_000,
    ):
        self.device = device
        self.baudrate = baudrate
        self.period_ns = period_ns
        self.max_age_ns = max_age_ns

        self.serial = None
        self.quantizer = AngularMotionQuantizer()

        self.next_slot_ns = None
        self.last_tx_ns = None
        self.last_measurement_ns = None
        self.rate_x = 0.0
        self.rate_y = 0.0
        self.quality = 0

    def open(self):
        import serial
        self.serial = serial.Serial(
            self.device,
            self.baudrate,
            timeout=0,
            write_timeout=0.01,
        )
        self.next_slot_ns = None
        self.last_tx_ns = None
        self.last_measurement_ns = None
        self.quality = 0
        self.quantizer.reset()

    def close(self):
        port = self.serial
        self.serial = None
        if port is not None:
            port.close()
        self.invalidate()
        self.next_slot_ns = None

    def invalidate(self):
        """Explicit optical-flow dropout."""
        self.last_measurement_ns = None
        self.last_tx_ns = None
        self.rate_x = 0.0
        self.rate_y = 0.0
        self.quality = 0
        self.quantizer.reset()

    def update(self, timestamp_ns, rate_x, rate_y, quality):
        if not all(math.isfinite(v) for v in (rate_x, rate_y)):
            raise ValueError("Non-finite optical flow")

        if not isinstance(timestamp_ns, int):
            raise ValueError("Invalid timestamp")

        if not isinstance(quality, int) or not 0 <= quality <= 255:
            raise ValueError("Invalid quality")

        self.last_measurement_ns = timestamp_ns
        self.rate_x = rate_x
        self.rate_y = rate_y
        self.quality = quality

    def tick(self, now_ns=None):
        if self.serial is None:
            raise RuntimeError("UART is not open")

        if now_ns is None:
            now_ns = time.monotonic_ns()

        if self.next_slot_ns is None:
            self.next_slot_ns = now_ns

        if now_ns < self.next_slot_ns:
            return False

        missed = max(
            0,
            (now_ns - self.next_slot_ns) // self.period_ns,
        )
        self.next_slot_ns += (missed + 1) * self.period_ns

        if self.last_measurement_ns is None:
            return False

        age_ns = now_ns - self.last_measurement_ns

        if age_ns < 0 or age_ns > self.max_age_ns:
            self.last_tx_ns = None
            self.quantizer.reset()
            return False

        if self.quality == 0:
            self.last_tx_ns = None
            self.quantizer.reset()
            return False

        if self.last_tx_ns is None:
            self.last_tx_ns = now_ns
            return False

        dt_ns = now_ns - self.last_tx_ns

        if dt_ns <= 0 or dt_ns >= 1_000_000_000:
            self.last_tx_ns = now_ns
            self.quantizer.reset()
            return False

        dt_s = dt_ns * 1e-9

        motion_x, motion_y = self.quantizer.encode(
            self.rate_x,
            self.rate_y,
            dt_s,
        )

        frame = opticalflow_packet(
            motion_x,
            motion_y,
            self.quality,
        )

        try:
            written = self.serial.write(frame)
            if written != len(frame):
                raise IOError(
                    f"Incomplete MSP frame write: {written}/{len(frame)}"
                )
        except Exception:
            try:
                self.close()
            except Exception:
                self.serial = None
                self.invalidate()
                self.next_slot_ns = None
            raise

        self.last_tx_ns = now_ns
        return True
