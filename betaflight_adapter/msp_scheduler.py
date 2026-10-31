#!/usr/bin/env python3
"""
JT-Zero MSPv2 offline publication scheduler.

No UART. No FC. No camera. No rangefinder.

One publication opportunity per period.
No backlog replay.
No stale-measurement publication.

The first opportunity after a discontinuity is
suppressed to establish a new timing baseline.

This is a scheduling prototype, NOT a flight-ready bridge.
"""

from dataclasses import dataclass


@dataclass
class SchedulerStats:
    published: int = 0
    stale: int = 0
    skipped_slots: int = 0
    suppressed: int = 0
    rejected: int = 0


class OfflineMspScheduler:

    def __init__(
        self,
        period_ns=20_000_000,
        max_age_ns=30_000_000,
    ):
        if period_ns <= 0:
            raise ValueError("Invalid publication period")

        if max_age_ns <= 0:
            raise ValueError("Invalid maximum age")

        self.period_ns = period_ns
        self.max_age_ns = max_age_ns

        self.stats = SchedulerStats()

        self.next_slot_ns = None
        self.last_measurement_end_ns = None

        self.need_baseline = True

    def reset(self):
        self.last_measurement_end_ns = None
        self.need_baseline = True

    def offer(
        self,
        now_ns,
        measurement_start_ns,
        measurement_end_ns,
        payload,
    ):
        """
        Returns payload if publication is allowed,
        otherwise None.

        No transmission occurs.
        """

        if not all(
            isinstance(v, int)
            for v in (
                now_ns,
                measurement_start_ns,
                measurement_end_ns,
            )
        ):
            self.stats.rejected += 1
            return None

        if (
            measurement_start_ns < 0
            or measurement_end_ns <= measurement_start_ns
            or now_ns < measurement_end_ns
        ):
            self.stats.rejected += 1
            return None

        if self.next_slot_ns is None:
            self.next_slot_ns = now_ns

        if now_ns < self.next_slot_ns:
            return None

        # Advance directly to the next future slot.
        # Never replay missed publication opportunities.
        if now_ns > self.next_slot_ns:
            missed = (
                (now_ns - self.next_slot_ns)
                // self.period_ns
            )

            if missed:
                self.stats.skipped_slots += missed
                self.reset()

            self.next_slot_ns += (
                (missed + 1) * self.period_ns
            )
        else:
            self.next_slot_ns += self.period_ns

        age_ns = now_ns - measurement_end_ns

        if age_ns > self.max_age_ns:
            self.stats.stale += 1
            self.reset()
            return None

        # Reject old, overlapping or duplicated measurements.
        if self.last_measurement_end_ns is not None:
            if measurement_start_ns < self.last_measurement_end_ns:
                self.stats.rejected += 1
                self.reset()
                return None

            if measurement_start_ns > self.last_measurement_end_ns:
                self.reset()

        self.last_measurement_end_ns = measurement_end_ns

        # Suppress first packet after a reset.
        if self.need_baseline:
            self.need_baseline = False
            self.stats.suppressed += 1
            return None

        self.stats.published += 1
        return payload
