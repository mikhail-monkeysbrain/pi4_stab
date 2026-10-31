#!/usr/bin/env python3
"""
JT-Zero / Betaflight offline MSPv2 pipeline.

OFFLINE ONLY:
- no UART
- no FC
- no camera
- no rangefinder
- no MonkeysStab runtime modification

Input: angular displacement over a measurement interval.
Output: MSPv2 bytes, or None when publication is suppressed.

Not flight-ready.
"""

import math
from dataclasses import dataclass

from msp_encoder import (
    AngularMotionQuantizer,
    opticalflow_packet,
    rangefinder_packet,
)
from msp_scheduler import OfflineMspScheduler


@dataclass
class PipelineStats:
    accepted: int = 0
    published: int = 0
    suppressed: int = 0
    rejected: int = 0
    resets: int = 0


class OfflineMspPipeline:

    def __init__(
        self,
        period_ns=20_000_000,
        max_age_ns=30_000_000,
    ):
        self.scheduler = OfflineMspScheduler(
            period_ns=period_ns,
            max_age_ns=max_age_ns,
        )

        self.quantizer = AngularMotionQuantizer()
        self.stats = PipelineStats()

        self.high_watermark_ns = None
        self.last_published_end_ns = None

    def reset_quantizer(self):
        self.quantizer.reset()
        self.last_published_end_ns = None
        self.stats.resets += 1

    def publish(
        self,
        now_ns,
        start_ns,
        end_ns,
        angle_x_rad,
        angle_y_rad,
        distance_m=None,
        quality=255,
    ):
        """
        Return (flow_frame, range_frame), or (None, None).

        No hardware transmission.
        """

        timestamps = (now_ns, start_ns, end_ns)

        if not all(
            isinstance(v, int) and not isinstance(v, bool)
            for v in timestamps
        ):
            self.stats.rejected += 1
            self.reset_quantizer()
            return None, None

        if not (
            0 <= start_ns < end_ns <= now_ns
            and end_ns - start_ns < 1_000_000_000
        ):
            self.stats.rejected += 1
            self.reset_quantizer()
            return None, None

        if not (
            math.isfinite(angle_x_rad)
            and math.isfinite(angle_y_rad)
            and isinstance(quality, int)
            and not isinstance(quality, bool)
            and 0 < quality <= 255
        ):
            self.stats.rejected += 1
            self.reset_quantizer()
            return None, None

        if self.high_watermark_ns is not None:
            if start_ns < self.high_watermark_ns:
                self.stats.rejected += 1
                self.reset_quantizer()
                return None, None

        dt_s = (end_ns - start_ns) * 1e-9

        rate_x = angle_x_rad / dt_s
        rate_y = angle_y_rad / dt_s

        if not (
            math.isfinite(rate_x)
            and math.isfinite(rate_y)
            and math.hypot(rate_x, rate_y) < 4.0
        ):
            self.stats.rejected += 1
            self.reset_quantizer()
            return None, None

        # This interval must not be accepted twice,
        # even if the scheduler suppresses it.
        self.high_watermark_ns = end_ns
        self.stats.accepted += 1

        # IMPORTANT:
        # Scheduler runs BEFORE the quantizer.
        decision = self.scheduler.offer(
            now_ns=now_ns,
            measurement_start_ns=start_ns,
            measurement_end_ns=end_ns,
            payload=True,
        )

        if decision is None:
            self.stats.suppressed += 1
            self.reset_quantizer()
            return None, None

        # No accumulation across unpublished intervals.
        if self.last_published_end_ns is not None:
            if start_ns != self.last_published_end_ns:
                self.reset_quantizer()

        mx, my = self.quantizer.encode(
            rate_x,
            rate_y,
            dt_s,
        )

        flow_frame = opticalflow_packet(
            mx,
            my,
            quality,
        )

        range_frame = None

        if distance_m is not None:
            if (
                math.isfinite(distance_m)
                and 0.08 <= distance_m <= 8.0
            ):
                range_frame = rangefinder_packet(
                    distance_m,
                    quality,
                )

        self.last_published_end_ns = end_ns
        self.stats.published += 1

        return flow_frame, range_frame
