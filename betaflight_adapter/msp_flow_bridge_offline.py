#!/usr/bin/env python3
"""Offline Optical Flow -> MSPv2 bridge. Never opens UART."""

import math

from msp_flow_accumulator import FlowAccumulator
from msp_scheduler import OfflineMspScheduler
from msp_encoder import AngularMotionQuantizer, opticalflow_packet


class OfflineFlowBridge:
    def __init__(self, period_ns=20_000_000, tolerance_ns=2_000_000):
        if period_ns <= 0 or tolerance_ns < 0:
            raise ValueError("Invalid timing configuration")

        self.period_ns = period_ns
        self.tolerance_ns = tolerance_ns

        self.acc = FlowAccumulator(max_gap_ns=0)
        self.scheduler = OfflineMspScheduler(
            period_ns=period_ns,
            max_age_ns=30_000_000,
        )
        self.quantizer = AngularMotionQuantizer()

        self.last_input_end_ns = None
        self.last_publish_ns = None
        self.need_sync = True

    def reset(self):
        self.acc.reset()
        self.scheduler.reset()
        self.quantizer.reset()
        self.last_input_end_ns = None
        self.last_publish_ns = None
        self.need_sync = True

    def offer(self, now_ns, start_ns, end_ns,
              rate_x, rate_y, quality):

        if not all(isinstance(v, int)
                   for v in (now_ns, start_ns, end_ns)):
            self.reset()
            return None

        if start_ns < 0 or end_ns <= start_ns or now_ns < end_ns:
            self.reset()
            return None

        if not all(math.isfinite(v) for v in (rate_x, rate_y)):
            self.reset()
            return None

        if not isinstance(quality, int) or not 1 <= quality <= 255:
            self.reset()
            return None

        # No overlap, duplication or unmeasured gap.
        if self.last_input_end_ns is not None:
            if start_ns != self.last_input_end_ns:
                self.reset()
                return None

        measured_ns = end_ns - start_ns

        if abs(measured_ns - self.period_ns) > self.tolerance_ns:
            self.reset()
            return None

        if not self.acc.add(start_ns, end_ns,
                            rate_x, rate_y, quality):
            self.reset()
            return None

        self.last_input_end_ns = end_ns

        # This prototype accepts one measured interval per slot.
        # Never accumulate an unbounded backlog.
        if (self.scheduler.next_slot_ns is not None
                and now_ns < self.scheduler.next_slot_ns):
            self.reset()
            return None

        window = self.acc.drain()

        if window is None:
            return None

        accepted = self.scheduler.offer(
            now_ns,
            window.start_ns,
            window.end_ns,
            window,
        )

        if accepted is None:
            self.quantizer.reset()
            self.last_publish_ns = None
            self.need_sync = True
            return None

        # A delayed slot must not be represented as a normal
        # 20 ms measurement.
        if self.last_publish_ns is not None:
            tx_dt_ns = now_ns - self.last_publish_ns

            if abs(tx_dt_ns - self.period_ns) > self.tolerance_ns:
                self.reset()
                return None

        if self.need_sync:
            self.quantizer.reset()
            self.need_sync = False
            self.last_publish_ns = now_ns

            # Synchronization only: no valid flow measurement.
            return opticalflow_packet(0, 0, 0)

        dt_s = accepted.dt_s

        rate_x = accepted.angle_x / dt_s
        rate_y = accepted.angle_y / dt_s

        motion_x, motion_y = self.quantizer.encode(
            rate_x, rate_y, dt_s
        )

        self.last_publish_ns = now_ns

        return opticalflow_packet(
            motion_x, motion_y, accepted.quality
        )
