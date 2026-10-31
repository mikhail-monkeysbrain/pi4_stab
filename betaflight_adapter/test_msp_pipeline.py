#!/usr/bin/env python3
"""Offline MSP pipeline integration tests."""

import struct

from msp_encoder import decode_frame
from msp_pipeline import OfflineMspPipeline

MS = 1_000_000


def send(p, now, start, end, angle=0.003):
    return p.publish(
        now_ns=now * MS,
        start_ns=start * MS,
        end_ns=end * MS,
        angle_x_rad=angle,
        angle_y_rad=0.0,
        distance_m=1.5,
        quality=255,
    )


def motion_x(result):
    flow, _ = result
    if flow is None:
        return None

    _, command, payload = decode_frame(flow)
    assert command == 0x1F02

    quality, mx, my = struct.unpack("<Bii", payload)
    assert quality == 255
    assert my == 0

    return mx


def main():
    p = OfflineMspPipeline()

    print("===== JT-ZERO MSP PIPELINE =====")
    print("UART: DISABLED")
    print()

    # Baseline: no output and no quantizer residue.
    assert send(p, 20, 0, 20)[0] is None
    assert p.quantizer.residual_x == 0
    print("PASS: baseline does not change quantizer")

    # 0.003 rad * 200 = 0.6 counts.
    assert motion_x(send(p, 40, 20, 40)) == 0
    assert motion_x(send(p, 60, 40, 60)) == 1
    print("PASS: fractional accumulation")

    # Duplicate interval must not be published.
    before = p.stats.published
    assert send(p, 80, 40, 60)[0] is None
    assert p.stats.published == before
    assert p.quantizer.residual_x == 0
    print("PASS: duplicate rejected and quantizer reset")

    # Recovery after rejected input.
    assert send(p, 100, 80, 100)[0] is None
    assert p.quantizer.residual_x == 0
    print("PASS: recovery baseline suppressed")

    assert motion_x(send(p, 120, 100, 120)) == 0
    print("PASS: clean publication after recovery")

    # Delayed wake-up, no replay of old packets.
    assert send(p, 220, 200, 220)[0] is None
    assert p.quantizer.residual_x == 0
    print("PASS: delayed wake-up suppressed")

    assert motion_x(send(p, 240, 220, 240)) == 0
    print("PASS: publication resumed without old residue")

    # Stale measurement.
    assert send(p, 320, 240, 260)[0] is None
    assert p.quantizer.residual_x == 0
    print("PASS: stale data suppressed")

    # Stale interval cannot be replayed.
    assert send(p, 340, 240, 260)[0] is None
    print("PASS: stale interval replay rejected")

    # Fresh data establishes baseline again.
    assert send(p, 360, 340, 360)[0] is None
    assert motion_x(send(p, 380, 360, 380)) == 0
    print("PASS: clean recovery after stale data")

    print()
    print("===== STATISTICS =====")
    print("Accepted:", p.stats.accepted)
    print("Published:", p.stats.published)
    print("Suppressed:", p.stats.suppressed)
    print("Rejected:", p.stats.rejected)
    print("Quantizer resets:", p.stats.resets)

    assert p.stats.accepted == 10
    assert p.stats.published == 5
    assert p.stats.suppressed == 5
    assert p.stats.rejected == 2

    print()
    print("PASS: Offline MSP pipeline integration")
    print("UART: DISABLED")


if __name__ == "__main__":
    main()
