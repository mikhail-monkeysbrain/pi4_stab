#!/usr/bin/env python3
"""Offline camera-interval to fixed-duration flow windows.

No UART, FC, camera or automatic hardware access.
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


class FlowWindowizer:
    def __init__(self, period_ns=20_000_000):
        if not isinstance(period_ns, int) or period_ns <= 0:
            raise ValueError("Invalid period")
        self.period_ns = period_ns
        self.reset()

    def reset(self):
        self.window_start_ns = None
        self.window_end_ns = None
        self.last_end_ns = None
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.quality = 255

    def add(self, start_ns, end_ns, rate_x, rate_y, quality):
        """Return zero or more completed, non-overlapping windows.

        Rates are assumed constant within each measured interval.
        Time is monotonic nanoseconds.
        """
        if (
            not isinstance(start_ns, int)
            or not isinstance(end_ns, int)
            or start_ns < 0
            or end_ns <= start_ns
            or not all(math.isfinite(v) for v in (rate_x, rate_y))
            or not isinstance(quality, int)
            or not 1 <= quality <= 255
        ):
            self.reset()
            return []

        if self.last_end_ns is not None:
            if start_ns != self.last_end_ns:
                self.reset()
                return []

        if self.window_start_ns is None:
            self.window_start_ns = start_ns
            self.window_end_ns = start_ns + self.period_ns

        result = []
        cursor = start_ns

        while cursor < end_ns:
            segment_end = min(end_ns, self.window_end_ns)
            dt_s = (segment_end - cursor) * 1e-9

            self.angle_x += rate_x * dt_s
            self.angle_y += rate_y * dt_s
            self.quality = min(self.quality, quality)

            cursor = segment_end

            if cursor == self.window_end_ns:
                result.append(
                    FlowWindow(
                        self.window_start_ns,
                        self.window_end_ns,
                        self.angle_x,
                        self.angle_y,
                        self.quality,
                    )
                )

                self.window_start_ns = self.window_end_ns
                self.window_end_ns += self.period_ns
                self.angle_x = 0.0
                self.angle_y = 0.0
                self.quality = 255

        self.last_end_ns = end_ns
        return result
