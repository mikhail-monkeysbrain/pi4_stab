#!/usr/bin/env python3
"""JT-Zero MSPv2 continuous-angle accumulator. OFFLINE ONLY."""

import math
from msp_encoder import AngularMotionQuantizer, opticalflow_packet, rangefinder_packet


class OfflineMspPipelineV2:
    def __init__(self, period_ns=20_000_000, max_age_ns=30_000_000):
        if period_ns <= 0 or max_age_ns <= 0:
            raise ValueError("Invalid timing configuration")
        self.period_ns = period_ns
        self.max_age_ns = max_age_ns
        self.quantizer = AngularMotionQuantizer()
        self.discarded_angle_x = 0.0
        self.discarded_angle_y = 0.0
        self.published_angle_x = 0.0
        self.published_angle_y = 0.0
        self.reset()

    def reset(self):
        if hasattr(self, 'angle_x'):
            self.discarded_angle_x += self.angle_x
            self.discarded_angle_y += self.angle_y
        self.quantizer.reset()
        self.start_ns = None
        self.end_ns = None
        self.next_slot_ns = None
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.count = 0

    def publish(self, now_ns, start_ns, end_ns,
                angle_x_rad, angle_y_rad,
                distance_m=None, quality=255):

        timestamps = (now_ns, start_ns, end_ns)
        if not all(type(v) is int for v in timestamps):
            self.reset()
            return None, None

        if not (0 <= start_ns < end_ns <= now_ns):
            self.reset()
            return None, None

        if end_ns - start_ns >= 1_000_000_000:
            self.reset()
            return None, None

        if now_ns - end_ns > self.max_age_ns:
            self.reset()
            return None, None

        if not (
            math.isfinite(angle_x_rad)
            and math.isfinite(angle_y_rad)
            and type(quality) is int
            and 0 < quality <= 255
        ):
            self.reset()
            return None, None

        dt_s = (end_ns - start_ns) * 1e-9
        if math.hypot(angle_x_rad / dt_s,
                      angle_y_rad / dt_s) >= 4.0:
            self.reset()
            return None, None

        # Never bridge an unknown or overlapping camera interval.
        if self.end_ns is not None and start_ns != self.end_ns:
            self.reset()

        if self.start_ns is None:
            self.start_ns = start_ns
            self.next_slot_ns = now_ns + self.period_ns

        self.end_ns = end_ns
        self.angle_x += angle_x_rad
        self.angle_y += angle_y_rad
        self.count += 1

        # Keep all valid displacement until the publication opportunity.
        if now_ns < self.next_slot_ns:
            return None, None

        window_s = (self.end_ns - self.start_ns) * 1e-9

        # No replay: one packet for the entire contiguous window.
        rate_x = self.angle_x / window_s
        rate_y = self.angle_y / window_s

        mx, my = self.quantizer.encode(rate_x, rate_y, window_s)
        self.published_angle_x += self.angle_x
        self.published_angle_y += self.angle_y
        flow_frame = opticalflow_packet(mx, my, quality)

        range_frame = None
        if distance_m is not None and math.isfinite(distance_m):
            if 0.08 <= distance_m <= 8.0:
                range_frame = rangefinder_packet(distance_m, quality)

        # Preserve fractional quantization residual.
        self.start_ns = None
        self.end_ns = None
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.count = 0
        self.next_slot_ns = None

        return flow_frame, range_frame
