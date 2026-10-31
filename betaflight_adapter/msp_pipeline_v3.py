#!/usr/bin/env python3
"""JT-Zero MSPv2 offline time-aligned pipeline. No hardware I/O."""

import math
from msp_encoder import (
    AngularMotionQuantizer,
    opticalflow_packet,
    rangefinder_packet,
)


class OfflineMspPipelineV3:
    def __init__(self, period_ns=20_000_000, max_age_ns=30_000_000):
        self.period_ns = period_ns
        self.max_age_ns = max_age_ns
        self.quantizer = AngularMotionQuantizer()
        self.reset()

    def reset(self):
        self.quantizer.reset()
        self.last_end_ns = None
        self.last_publish_ns = None
        self.angle_x = 0.0
        self.angle_y = 0.0

    def publish(self, now_ns, start_ns, end_ns,
                angle_x_rad, angle_y_rad,
                distance_m=None, quality=255):

        if not all(type(v) is int for v in (now_ns, start_ns, end_ns)):
            self.reset()
            return None, None

        if not (0 <= start_ns < end_ns <= now_ns):
            self.reset()
            return None, None

        if now_ns - end_ns > self.max_age_ns:
            self.reset()
            return None, None

        if not (
            math.isfinite(angle_x_rad)
            and math.isfinite(angle_y_rad)
            and 0 < quality <= 255
        ):
            self.reset()
            return None, None

        if self.last_end_ns is not None and start_ns != self.last_end_ns:
            self.reset()

        self.last_end_ns = end_ns
        self.angle_x += angle_x_rad
        self.angle_y += angle_y_rad

        if self.last_publish_ns is None:
            self.last_publish_ns = now_ns
            self.angle_x = 0.0
            self.angle_y = 0.0
            return None, None

        elapsed_ns = now_ns - self.last_publish_ns

        if elapsed_ns < self.period_ns:
            return None, None

        dt_s = elapsed_ns * 1e-9

        mx, my = self.quantizer.encode(
            self.angle_x / dt_s,
            self.angle_y / dt_s,
            dt_s,
        )

        flow_frame = opticalflow_packet(mx, my, quality)

        range_frame = None
        if distance_m is not None and math.isfinite(distance_m):
            if 0.08 <= distance_m <= 8.0:
                range_frame = rangefinder_packet(distance_m, quality)

        self.angle_x = 0.0
        self.angle_y = 0.0
        self.last_publish_ns = now_ns

        return flow_frame, range_frame
