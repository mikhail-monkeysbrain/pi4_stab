#!/usr/bin/env python3

"""Offline scheduler safety tests. No UART or hardware."""

from msp_scheduler import OfflineMspScheduler

MS = 1_000_000


def offer(s, now, start, end, payload):
    return s.offer(
        now_ns=now * MS,
        measurement_start_ns=start * MS,
        measurement_end_ns=end * MS,
        payload=payload,
    )


def main():
    s = OfflineMspScheduler()

    # First measurement establishes timing baseline.
    assert offer(s, 20, 0, 20, "A") is None
    print("PASS: initial baseline suppressed")

    assert offer(s, 40, 20, 40, "B") == "B"
    assert offer(s, 60, 40, 60, "C") == "C"
    print("PASS: regular publication")

    # No second publication inside the same slot.
    assert offer(s, 65, 40, 60, "D") is None
    print("PASS: no duplicate publication inside slot")

    # Scheduler resumes at 160 ms after a 100 ms pause.
    # Missed slots must not be replayed.
    assert offer(s, 160, 140, 160, "E") is None
    print("PASS: delayed wake-up, no backlog replay")

    assert offer(s, 180, 160, 180, "F") == "F"
    print("PASS: publication resumes")

    # Stale measurement: end at 180, now 240.
    assert offer(s, 240, 160, 180, "OLD") is None
    print("PASS: stale measurement rejected")

    assert offer(s, 260, 240, 260, "G") is None
    assert offer(s, 280, 260, 280, "H") == "H"
    print("PASS: recovery after stale data")

    print()
    print("===== STATISTICS =====")
    print("Published:", s.stats.published)
    print("Stale:", s.stats.stale)
    print("Skipped slots:", s.stats.skipped_slots)
    print("Suppressed:", s.stats.suppressed)
    print("Rejected:", s.stats.rejected)

    assert s.stats.published == 4
    assert s.stats.stale == 1
    assert s.stats.skipped_slots == 6
    assert s.stats.suppressed == 3

    print()
    print("PASS: Offline MSP scheduler")
    print("UART: DISABLED")


if __name__ == "__main__":
    main()
