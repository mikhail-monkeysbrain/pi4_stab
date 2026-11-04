#include "bf_flow_msp_tx_scheduler.hpp"

#include <cstdint>
#include <iostream>

using bf_flow::MspAggregator;
using bf_flow::MspTxScheduler;

static MspAggregator::Result window(
    int index,
    int32_t x,
    int32_t y
) {
    MspAggregator::Result r;
    r.start_ns = 1000000000LL +
        int64_t(index) * 50000000LL;
    r.end_ns = r.start_ns + 50000000LL;
    r.coverage_ns = 50000000LL;
    r.x = x;
    r.y = y;
    r.valid = true;
    return r;
}

int main() {
    MspTxScheduler scheduler;
    MspTxScheduler::Packet packet;

    bool ok = true;

    scheduler.add(window(0, 10, -5));
    scheduler.add(window(1, 10, -5));

    const bool continuous =
        scheduler.build(packet) &&
        packet.x == 20 &&
        packet.y == -10 &&
        packet.windows == 2 &&
        packet.interval_ns == 100000000LL;

    std::cout << "CONTINUOUS="
              << (continuous ? "PASS" : "FAIL")
              << '\n';

    ok &= continuous;

    scheduler.reset();
    scheduler.add(window(0, 10, 0));

    const bool gap_rejected =
        !scheduler.add(window(2, 10, 0));

    const bool no_stale_packet =
        !scheduler.build(packet);

    const bool dropout =
        gap_rejected && no_stale_packet;

    std::cout << "DROPOUT="
              << (dropout ? "PASS" : "FAIL")
              << '\n';

    ok &= dropout;

    std::cout << "BF_TX_SCHEDULER_TEST_RESULT="
              << (ok ? "PASS" : "FAIL")
              << '\n';

    return ok ? 0 : 1;
}
