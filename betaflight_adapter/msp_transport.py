#!/usr/bin/env python3
"""
JT-Zero / Betaflight MSPv2 offline transport prototype.

OFFLINE ONLY:
- No UART
- No camera
- No rangefinder
- No flight-controller communication

Input:
    Angular displacement in radians over a valid measurement interval.

Output:
    Encoded MSPv2 packets and diagnostic statistics.

This prototype does not establish FC receive timing.
It is NOT flight-ready.
"""

import argparse
import math
import struct
from dataclasses import dataclass

from msp_encoder import (
    AngularMotionQuantizer,
    decode_frame,
    opticalflow_packet,
    rangefinder_packet,
)


@dataclass
class TransportStats:
    flow_packets: int = 0
    range_packets: int = 0
    rejected: int = 0
    discontinuities: int = 0
    motion_x_total: int = 0
    motion_y_total: int = 0


class OfflineMspTransport:

    def __init__(self):
        self.quantizer = AngularMotionQuantizer()
        self.stats = TransportStats()
        self.last_end_ns = None
        self.high_watermark_ns = None

    def discontinuity(self):
        self.quantizer.reset()
        self.last_end_ns = None
        self.stats.discontinuities += 1

    def publish(
        self,
        start_ns,
        end_ns,
        angle_x_rad,
        angle_y_rad,
        distance_m=None,
        quality=255,
    ):
        """
        Accept one contiguous measurement interval.

        Returns:
            (flow_frame, range_frame)

        No data is transmitted.
        """

        if not isinstance(start_ns, int):
            raise TypeError("start_ns must be int")

        if not isinstance(end_ns, int):
            raise TypeError("end_ns must be int")

        dt_ns = end_ns - start_ns

        valid = (
            start_ns >= 0
            and 0 < dt_ns < 1_000_000_000
            and math.isfinite(angle_x_rad)
            and math.isfinite(angle_y_rad)
            and isinstance(quality, int)
            and 0 <= quality <= 255
        )

        if not valid:
            self.stats.rejected += 1
            self.discontinuity()
            return None, None

        # Never accept an interval that overlaps or precedes
        # any previously accepted measurement, even after reset.
        if self.high_watermark_ns is not None:
            if start_ns < self.high_watermark_ns:
                self.stats.rejected += 1
                self.discontinuity()
                return None, None

        # A forward gap starts a new independent segment.
        if self.last_end_ns is not None:
            if start_ns > self.last_end_ns:
                self.discontinuity()

        dt_s = dt_ns * 1e-9

        rate_x = angle_x_rad / dt_s
        rate_y = angle_y_rad / dt_s

        if not math.isfinite(rate_x) or not math.isfinite(rate_y):
            self.stats.rejected += 1
            self.discontinuity()
            return None, None

        # Keep the prototype within a conservative angular-rate bound.
        if math.hypot(rate_x, rate_y) >= 4.0:
            self.stats.rejected += 1
            self.discontinuity()
            return None, None

        if quality == 0:
            self.stats.rejected += 1
            self.discontinuity()
            return None, None

        # Quantize integrated angular displacement.
        motion_x, motion_y = self.quantizer.encode(
            rate_x,
            rate_y,
            dt_s,
        )

        flow_frame = opticalflow_packet(
            motion_x,
            motion_y,
            quality,
        )

        # Validate the encoded frame.
        _, command, payload = decode_frame(flow_frame)
        assert command == 0x1F02

        decoded_quality, decoded_x, decoded_y = struct.unpack(
            "<Bii",
            payload,
        )

        assert decoded_quality == quality
        assert decoded_x == motion_x
        assert decoded_y == motion_y

        range_frame = None

        if distance_m is not None:
            if math.isfinite(distance_m) and 0.08 <= distance_m <= 8.0:
                range_frame = rangefinder_packet(distance_m, quality)

                _, range_command, range_payload = decode_frame(
                    range_frame
                )

                assert range_command == 0x1F01

                range_quality, distance_mm = struct.unpack(
                    "<Bi",
                    range_payload,
                )

                assert range_quality == quality
                assert distance_mm > 0

                self.stats.range_packets += 1

        self.last_end_ns = end_ns
        self.high_watermark_ns = end_ns

        self.stats.flow_packets += 1
        self.stats.motion_x_total += motion_x
        self.stats.motion_y_total += motion_y

        return flow_frame, range_frame


def offline_demo():

    transport = OfflineMspTransport()

    period_ns = 20_000_000

    print("===== JT-ZERO MSPv2 TRANSPORT =====")
    print("MODE: OFFLINE")
    print("UART: DISABLED")
    print("FC: NOT CONNECTED")
    print("CAMERA: NOT REQUIRED")
    print("RANGEFINDER: NOT REQUIRED")
    print()

    print("step,flow_bytes,range_bytes,motion_x,motion_y")

    for step in range(100):

        start_ns = step * period_ns
        end_ns = start_ns + period_ns

        # Artificial dropout: 10 intervals.
        if 40 <= step < 50:
            if step == 40:
                transport.discontinuity()
            continue

        dt_s = period_ns * 1e-9

        # Artificial raw angular flow.
        angle_x = 0.15 * dt_s
        angle_y = -0.08 * dt_s

        flow_frame, range_frame = transport.publish(
            start_ns=start_ns,
            end_ns=end_ns,
            angle_x_rad=angle_x,
            angle_y_rad=angle_y,
            distance_m=1.50,
            quality=255,
        )

        if flow_frame is None:
            continue

        _, _, payload = decode_frame(flow_frame)
        _, motion_x, motion_y = struct.unpack("<Bii", payload)

        print(
            f"{step},"
            f"{len(flow_frame)},"
            f"{len(range_frame) if range_frame else 0},"
            f"{motion_x},"
            f"{motion_y}"
        )

    stats = transport.stats

    print()
    print("===== STATISTICS =====")
    print("Flow packets:", stats.flow_packets)
    print("Range packets:", stats.range_packets)
    print("Rejected:", stats.rejected)
    print("Discontinuities:", stats.discontinuities)
    print("Motion X total:", stats.motion_x_total)
    print("Motion Y total:", stats.motion_y_total)

    assert stats.flow_packets == 90
    assert stats.range_packets == 90
    assert stats.rejected == 0
    assert stats.discontinuities == 1
    assert stats.motion_x_total == 54
    assert abs(stats.motion_y_total - (-28.8)) <= 2.0

    print()
    print("PASS: Offline MSPv2 transport")
    print("PASS: Packet encoding")
    print("PASS: Measurement discontinuity reset")
    print("PASS: Angular displacement accumulation")
    print()
    print("NOT VERIFIED: Real FC receive timing")
    print("NOT VERIFIED: Sensor axis orientation")
    print("NOT VERIFIED: Gyro compensation")
    print("NOT VERIFIED: Physical sensor accuracy")


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--offline-demo", action="store_true")
    args = parser.parse_args()

    if args.offline_demo:
        offline_demo()
    else:
        parser.print_help()
