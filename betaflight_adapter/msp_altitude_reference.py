#!/usr/bin/env python3
"""Betaflight MSP altitude reference for Optical Flow.

First valid altitude starts a 5-second initialization.
The first valid sample at or after 5 seconds becomes 0.10 m AGL.

No UART, hardware access or automatic tests.
"""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class AltitudeResult:
    ready: bool
    height_m: Optional[float]
    reference_m: Optional[float]
    elapsed_s: float
    reason: str


class MspAltitudeReference:
    def __init__(
        self,
        initial_height_m=0.10,
        calibration_s=5.0,
        max_age_s=0.15,
    ):
        if initial_height_m <= 0:
            raise ValueError("initial_height_m must be positive")

        if calibration_s <= 0:
            raise ValueError("calibration_s must be positive")

        if max_age_s <= 0:
            raise ValueError("max_age_s must be positive")

        self.initial_height_m = float(initial_height_m)
        self.calibration_ns = int(calibration_s * 1e9)
        self.max_age_ns = int(max_age_s * 1e9)

        self.reset()

    def reset(self):
        self.first_ns = None
        self.last_ns = None
        self.reference_m = None

    def update(
        self,
        altitude_m,
        sample_ns,
        now_ns,
    ):
        if (
            not isinstance(sample_ns, int)
            or not isinstance(now_ns, int)
            or sample_ns < 0
            or now_ns < sample_ns
            or not isinstance(altitude_m, (int, float))
            or not math.isfinite(altitude_m)
        ):
            self.reset()
            return AltitudeResult(
                False, None, None, 0.0, "INVALID_SAMPLE"
            )

        age_ns = now_ns - sample_ns

        if age_ns > self.max_age_ns:
            self.reset()
            return AltitudeResult(
                False, None, None, 0.0, "STALE_SAMPLE"
            )

        if self.last_ns is not None:
            if sample_ns <= self.last_ns:
                return AltitudeResult(
                    False,
                    None,
                    self.reference_m,
                    0.0,
                    "DUPLICATE_OR_OLD_SAMPLE",
                )

            if sample_ns - self.last_ns > self.max_age_ns:
                self.reset()

        self.last_ns = sample_ns

        if self.first_ns is None:
            self.first_ns = sample_ns

        elapsed_ns = sample_ns - self.first_ns
        elapsed_s = elapsed_ns * 1e-9

        if self.reference_m is None:
            if elapsed_ns < self.calibration_ns:
                return AltitudeResult(
                    False,
                    None,
                    None,
                    elapsed_s,
                    "CALIBRATING",
                )

            self.reference_m = float(altitude_m)

        height_m = (
            self.initial_height_m
            + float(altitude_m)
            - self.reference_m
        )

        if height_m <= 0:
            return AltitudeResult(
                False,
                None,
                self.reference_m,
                elapsed_s,
                "NON_POSITIVE_HEIGHT",
            )

        return AltitudeResult(
            True,
            height_m,
            self.reference_m,
            elapsed_s,
            "READY",
        )
