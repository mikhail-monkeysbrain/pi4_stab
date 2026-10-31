#!/usr/bin/env python3
"""
JT-Zero MSPv2 pipeline: deterministic offline stress test.

3000 synthetic intervals, 20 ms each (60 seconds).

No UART, FC, camera, rangefinder or real-time waiting.
No changes to the production MonkeysStab runtime.
"""

import math
import struct

from msp_encoder import decode_frame
from msp_pipeline import OfflineMspPipeline

PERIOD_NS = 20_000_000
COUNT = 3000
RATE_X = 0.15
RATE_Y = -0.08
DISTANCE_M = 1.5


def verify_packet(flow_frame, range_frame):
    assert flow_frame is not None
    assert range_frame is not None

    assert len(flow_frame) == 18
    assert len(range_frame) == 14

    _, flow_command, flow_payload = decode_frame(flow_frame)
    _, range_command, range_payload = decode_frame(range_frame)

    assert flow_command == 0x1F02
    assert range_command == 0x1F01

    fq, mx, my = struct.unpack("<Bii", flow_payload)
    rq, mm = struct.unpack("<Bi", range_payload)

    assert fq == 255
    assert rq == 255
    assert mm == 1500

    # Conservative bounds for this synthetic test.
    assert abs(mx) <= 2
    assert abs(my) <= 2

    return mx, my


def main():
    pipeline = OfflineMspPipeline(
        period_ns=PERIOD_NS,
        max_age_ns=30_000_000,
    )

    published_intervals = set()

    motion_x_total = 0
    motion_y_total = 0

    checked_packets = 0
    suppressed = 0

    injected = {
        "missing": 0,
        "delayed": 0,
        "duplicate": 0,
        "overlap": 0,
        "bad_quality": 0,
    }

    print("===== JT-ZERO MSPv2 STRESS TEST =====")
    print("Synthetic duration: 60 s")
    print("Synthetic frequency: 50 Hz")
    print("UART: DISABLED")
    print("FC: NOT CONNECTED")
    print()

    for i in range(COUNT):
        start_ns = i * PERIOD_NS
        end_ns = start_ns + PERIOD_NS
        now_ns = end_ns

        # Missing source intervals.
        if 500 <= i < 510 or 1800 <= i < 1810:
            injected["missing"] += 1
            continue

        # Simulated stale measurements.
        if 900 <= i < 910:
            now_ns += 60_000_000
            injected["delayed"] += 1

        # Simulated low-quality measurements.
        quality = 255
        if 1300 <= i < 1310:
            quality = 0
            injected["bad_quality"] += 1

        angle_x = RATE_X * (PERIOD_NS * 1e-9)
        angle_y = RATE_Y * (PERIOD_NS * 1e-9)

        result = pipeline.publish(
            now_ns=now_ns,
            start_ns=start_ns,
            end_ns=end_ns,
            angle_x_rad=angle_x,
            angle_y_rad=angle_y,
            distance_m=DISTANCE_M,
            quality=quality,
        )

        flow_frame, range_frame = result

        if flow_frame is None:
            suppressed += 1
            assert range_frame is None

            # Suppressed measurement must not retain
            # fractional quantizer state.
            assert abs(pipeline.quantizer.residual_x) < 1e-12
            assert abs(pipeline.quantizer.residual_y) < 1e-12
        else:
            interval = (start_ns, end_ns)

            assert interval not in published_intervals
            published_intervals.add(interval)

            mx, my = verify_packet(
                flow_frame,
                range_frame,
            )

            motion_x_total += mx
            motion_y_total += my
            checked_packets += 1

        # Replay a previously processed interval.
        if i in (200, 1100, 2200):
            injected["duplicate"] += 1

            old = pipeline.publish(
                now_ns=now_ns,
                start_ns=start_ns,
                end_ns=end_ns,
                angle_x_rad=angle_x,
                angle_y_rad=angle_y,
                distance_m=DISTANCE_M,
                quality=255,
            )

            assert old == (None, None)

            assert abs(pipeline.quantizer.residual_x) < 1e-12
            assert abs(pipeline.quantizer.residual_y) < 1e-12

        # Overlap the previous measurement.
        if i in (300, 1500, 2500):
            injected["overlap"] += 1

            old = pipeline.publish(
                now_ns=now_ns,
                start_ns=start_ns - 10_000_000,
                end_ns=end_ns,
                angle_x_rad=angle_x,
                angle_y_rad=angle_y,
                distance_m=DISTANCE_M,
                quality=255,
            )

            assert old == (None, None)

    stats = pipeline.stats
    scheduler = pipeline.scheduler.stats

    print("===== INJECTED EVENTS =====")
    for key, value in injected.items():
        print(f"{key}: {value}")

    print()
    print("===== PIPELINE =====")
    print("Accepted:", stats.accepted)
    print("Published:", stats.published)
    print("Suppressed:", stats.suppressed)
    print("Rejected:", stats.rejected)
    print("Quantizer resets:", stats.resets)

    print()
    print("===== SCHEDULER =====")
    print("Published:", scheduler.published)
    print("Stale:", scheduler.stale)
    print("Skipped slots:", scheduler.skipped_slots)
    print("Suppressed:", scheduler.suppressed)
    print("Rejected:", scheduler.rejected)

    print()
    print("===== MOTION =====")
    print("Verified frames:", checked_packets)
    print("Total motion X counts:", motion_x_total)
    print("Total motion Y counts:", motion_y_total)
    print("Total motion X rad:", motion_x_total / 200)
    print("Total motion Y rad:", motion_y_total / 200)

    # Accounting invariants.
    assert stats.published == checked_packets
    assert scheduler.published == stats.published
    assert stats.accepted == stats.published + stats.suppressed

    assert len(published_intervals) == checked_packets

    assert stats.rejected >= (
        injected["duplicate"] + injected["overlap"]
    )

    assert scheduler.stale >= injected["delayed"]

    assert checked_packets > 0
    assert checked_packets < COUNT

    assert math.isfinite(motion_x_total)
    assert math.isfinite(motion_y_total)

    print()
    print("PASS: 60-second synthetic sequence")
    print("PASS: No duplicate publication")
    print("PASS: No rejected-frame quantizer residue")
    print("PASS: MSPv2 framing and CRC")
    print("PASS: Pipeline/scheduler counters")
    print()
    print("NOT VERIFIED: Real-time scheduling")
    print("NOT VERIFIED: FC receive timing")
    print("NOT VERIFIED: Gyro and axis compatibility")
    print("UART: DISABLED")


if __name__ == "__main__":
    main()
