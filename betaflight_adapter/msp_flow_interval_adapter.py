#!/usr/bin/env python3
"""Convert validated optical-flow intervals into MSPv2 packets."""

import math

from msp_encoder import AngularMotionQuantizer, opticalflow_packet


class MspFlowIntervalAdapter:
    def __init__(self):
        self.quantizer = AngularMotionQuantizer()
        self.last_timestamp_ns = None

    def reset(self):
        self.quantizer.reset()
        self.last_timestamp_ns = None

    def encode(self, timestamp_ns, rate_x, rate_y, dt_s, quality):
        if not isinstance(timestamp_ns, int):
            self.reset()
            return None

        if self.last_timestamp_ns is not None:
            if timestamp_ns <= self.last_timestamp_ns:
                self.reset()
                return None

        if not all(math.isfinite(v) for v in (rate_x, rate_y, dt_s)):
            self.reset()
            return None

        if not 0.0 < dt_s < 0.2:
            self.reset()
            return None

        if not isinstance(quality, int) or not 1 <= quality <= 255:
            self.reset()
            return None

        motion_x, motion_y = self.quantizer.encode(
            rate_x,
            rate_y,
            dt_s,
        )

        frame = opticalflow_packet(
            motion_x,
            motion_y,
            quality,
        )

        self.last_timestamp_ns = timestamp_ns
        return frame
