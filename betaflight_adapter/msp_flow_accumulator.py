#!/usr/bin/env python3
"""Accumulate measured optical-flow angular displacement.

Offline component. Does not access UART or flight controller.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class FlowWindow:
    start_ns: int
    end_ns: int
    angle_x: float
    angle_y: float
    quality: int
    samples: int

    @property
    def dt_s(self):
        return (self.end_ns - self.start_ns) * 1e-9


class FlowAccumulator:
    def __init__(self, max_gap_ns=30_000_000):
        self.max_gap_ns = max_gap_ns
        self.reset()

    def reset(self):
        self.start_ns = None
        self.end_ns = None
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.quality = 255
        self.samples = 0

    def add(self, start_ns, end_ns, rate_x, rate_y, quality):
        """Add a measured camera interval.

        Returns False and resets on invalid or discontinuous input.
        """
        valid = (
            isinstance(start_ns, int)
            and isinstance(end_ns, int)
            and end_ns > start_ns
            and math.isfinite(rate_x)
            and math.isfinite(rate_y)
            and isinstance(quality, int)
            and 1 <= quality <= 255
        )

        if not valid:
            self.reset()
            return False

        if self.end_ns is not None:
            gap_ns = start_ns - self.end_ns

            if gap_ns < 0 or gap_ns > self.max_gap_ns:
                self.reset()
                return False

        dt_s = (end_ns - start_ns) * 1e-9

        if self.start_ns is None:
            self.start_ns = start_ns

        self.angle_x += rate_x * dt_s
        self.angle_y += rate_y * dt_s
        self.end_ns = end_ns
        self.quality = min(self.quality, quality)
        self.samples += 1

        return True

    def drain(self):
        """Consume the contiguous measured window."""
        if self.samples == 0:
            return None

        result = FlowWindow(
            start_ns=self.start_ns,
            end_ns=self.end_ns,
            angle_x=self.angle_x,
            angle_y=self.angle_y,
            quality=self.quality,
            samples=self.samples,
        )

        self.reset()
        return result
