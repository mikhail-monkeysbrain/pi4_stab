#!/usr/bin/env python3

"""Offline interval-safety tests. No UART or sensors."""

from msp_transport import OfflineMspTransport

NS = 1_000_000


def send(t, start_ms, end_ms):
    return t.publish(
        start_ns=start_ms * NS,
        end_ns=end_ms * NS,
        angle_x_rad=0.003,
        angle_y_rad=-0.002,
        distance_m=1.5,
        quality=255,
    )


def accepted(result):
    return result[0] is not None


def rejected(result):
    return result == (None, None)


def main():
    t = OfflineMspTransport()

    assert accepted(send(t, 0, 20))
    assert accepted(send(t, 20, 40))
    print("PASS: contiguous intervals")

    before = t.stats.motion_x_total
    assert rejected(send(t, 20, 40))
    assert t.stats.motion_x_total == before
    print("PASS: duplicate interval rejected")

    assert rejected(send(t, 10, 30))
    print("PASS: overlapping interval rejected")

    assert rejected(send(t, 0, 20))
    print("PASS: old interval rejected after reset")

    assert accepted(send(t, 60, 80))
    print("PASS: fresh interval accepted after gap")

    assert accepted(send(t, 100, 120))
    print("PASS: forward gap accepted as new segment")

    assert rejected(send(t, 80, 100))
    print("PASS: out-of-order interval rejected")

    assert accepted(send(t, 120, 140))
    print("PASS: forward progress restored")

    assert rejected(send(t, 140, 140))
    print("PASS: zero-duration interval rejected")

    assert rejected(send(t, 160, 1160))
    print("PASS: excessive-duration interval rejected")

    assert accepted(send(t, 1180, 1200))
    print("PASS: recovery after invalid interval")

    print()
    print("===== STATISTICS =====")
    print("Flow packets:", t.stats.flow_packets)
    print("Rejected:", t.stats.rejected)
    print("Discontinuities:", t.stats.discontinuities)
    print()
    print("PASS: Interval safety checks")
    print("UART: DISABLED")


if __name__ == "__main__":
    main()
